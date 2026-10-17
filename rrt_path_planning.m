   
function rrt_path_planning()
    % Initialize ROS
    rosinit;

    % Parameters
    start = [0, 0]; % Starting point (assume in map coordinates)
    goal = [4, 4]; % Goal point (assume in map coordinates)
    step_size = 0.2; % Step size for extending the tree
    max_iter = 1000; % Maximum number of iterations
    goal_radius = 0.2; % Radius around the goal to consider the goal reached
    obstacles = [2, 2, 0.5; 3, 3, 0.5]; % Circular obstacles [x, y, radius]

    % Spawn start and goal markers in Gazebo
    if ~spawn_model('start_marker.sdf', 'start_marker', [start 0])
        disp('Failed to spawn start marker');
        rosshutdown;
        return;
    end
    if ~spawn_model('goal_marker.sdf', 'goal_marker', [goal 0])
        disp('Failed to spawn goal marker');
        rosshutdown;
        return;
    end

    % Set up publisher for velocity commands
    velocityPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
    velocityMsg = rosmessage(velocityPub);

    % Set up subscriber for laser scan data
    laserSub = rossubscriber('/scan');
    
    % Set up subscriber for odometry data
    odomSub = rossubscriber('/odom', 'nav_msgs/Odometry');

    % Initialize tree with the starting point
    tree.vertices = start;
    tree.edges = [];

    % Initialize the figure for plotting
    figure;
    hold on;
    axis equal;
    xlim([-1 5]);
    ylim([-1 5]);
    plot_obstacles(obstacles);

    % Run the RRT algorithm
    for iter = 1:max_iter
        % Sample a random point
        rand_point = [rand*4, rand*4];

        % Find the nearest vertex in the tree
        [nearest_vertex, nearest_index] = find_nearest_vertex(tree.vertices, rand_point);

        % Extend the tree towards the random point
        new_vertex = extend_tree(nearest_vertex, rand_point, step_size);

        % Check for collision with obstacles
        if ~check_collision(nearest_vertex, new_vertex, obstacles)
            % Add the new vertex to the tree
            tree.vertices = [tree.vertices; new_vertex];
            tree.edges = [tree.edges; nearest_index, size(tree.vertices, 1)];

            % Plot the new edge
            plot([nearest_vertex(1), new_vertex(1)], [nearest_vertex(2), new_vertex(2)], 'b');
            drawnow;

            % Check if the goal is reached
            if norm(new_vertex - goal) < goal_radius
                disp('Goal reached!');
                break;
            end
        end
    end

    % Find and plot the path from start to goal
    path = find_path(tree, size(tree.vertices, 1));
    if ~isempty(path)
        plot_path(tree.vertices, path, start, goal);
    else
        disp('No valid path found.');
    end

    % Move the TurtleBot3 along the path
    follow_path(tree.vertices, path, velocityPub, velocityMsg, odomSub);

    % Shut down ROS
    rosshutdown;
end

function success = spawn_model(sdf_file, model_name, position)
    success = false;

    % Load the SDF file
    if ~isfile(sdf_file)
        disp(['SDF file not found: ', sdf_file]);
        return;
    end

    sdf_content = fileread(sdf_file);

    % Create ROS service client for spawning models
    spawnClient = rossvcclient('/gazebo/spawn_sdf_model');
    spawnReq = rosmessage(spawnClient);

    % Set the request parameters
    spawnReq.ModelName = model_name;
    spawnReq.ModelXml = sdf_content;
    spawnReq.InitialPose.Position.X = position(1);
    spawnReq.InitialPose.Position.Y = position(2);
    spawnReq.InitialPose.Position.Z = position(3);
    spawnReq.ReferenceFrame = 'world';

    % Call the service
    try
        call(spawnClient, spawnReq);
        success = true;
    catch ME
        disp(['Failed to spawn model: ', model_name]);
        disp(getReport(ME, 'extended'));
    end
end

function [nearest_vertex, nearest_index] = find_nearest_vertex(vertices, point)
    distances = vecnorm(vertices - point, 2, 2);
    [~, nearest_index] = min(distances);
    nearest_vertex = vertices(nearest_index, :);
end

function new_vertex = extend_tree(nearest_vertex, rand_point, step_size)
    direction = (rand_point - nearest_vertex) / norm(rand_point - nearest_vertex);
    new_vertex = nearest_vertex + step_size * direction;
end

function path = find_path(tree, goal_index)
    path = goal_index;
    while path(1) ~= 1
        found = false;
        for i = 1:size(tree.edges, 1)
            if tree.edges(i, 2) == path(1)
                path = [tree.edges(i, 1), path];
                found = true;
                break;
            end
        end
        if ~found
            path = []; % No valid path found
            break;
        end
    end
end

function collision = check_collision(v1, v2, obstacles)
    collision = false;
    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        if line_circle_collision(v1, v2, obstacle(1:2), obstacle(3))
            collision = true;
            return;
        end
    end
end

function collision = line_circle_collision(p1, p2, center, radius)
    v1 = p1 - center;
    v2 = p2 - center;
    a = dot(v2 - v1, v2 - v1);
    b = 2 * dot(v1, v2 - v1);
    c = dot(v1, v1) - radius^2;
    disc = b^2 - 4 * a * c;
    if disc >= 0
        t1 = (-b - sqrt(disc)) / (2 * a);
        t2 = (-b + sqrt(disc)) / (2 * a);
        if (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)
            collision = true;
            return;
        end
    end
    collision = false;
end

function plot_obstacles(obstacles)
    for i = 1:size(obstacles, 1)
        rectangle('Position', [obstacles(i, 1) - obstacles(i, 3), obstacles(i, 2) - obstacles(i, 3), ...
                               2*obstacles(i, 3), 2*obstacles(i, 3)], ...
                  'Curvature', [1, 1], 'FaceColor', [0.5, 0.5, 0.5]);
    end
end

function plot_path(vertices, path, start, goal)
    for i = 1:length(path)-1
        plot([vertices(path(i), 1), vertices(path(i+1), 1)], ...
             [vertices(path(i), 2), vertices(path(i+1), 2)], 'r', 'LineWidth', 2);
    end
    % Plot start and goal
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end

function follow_path(vertices, path, velocityPub, velocityMsg, odomSub)
    for i = 1:length(path)-1
        move_to_point(vertices(path(i), :), vertices(path(i+1), :), velocityPub, velocityMsg, odomSub);
    end

    % Stop the robot
    velocityMsg.Linear.X = 0;
    velocityMsg.Angular.Z = 0;
    send(velocityPub, velocityMsg);
end

function move_to_point(start_point, end_point, velocityPub, velocityMsg, odomSub)
    % Control the TurtleBot3 to move from start_point to end_point
    max_speed = 0.2;
    max_angular_speed = 1.0;
    goal_threshold = 0.1; % Threshold to consider the goal reached

    while true
        % Get current pose from odometry
        odomData = receive(odomSub);
        current_position = [odomData.Pose.Pose.Position.X, odomData.Pose.Pose.Position.Y];
        current_orientation = odomData.Pose.Pose.Orientation;
        angles = quat2eul([current_orientation.W, current_orientation.X, current_orientation.Y, current_orientation.Z]);
        current_angle = angles(1);

        % Calculate distance and angle to the goal
        dx = end_point(1) - current_position(1);
        dy = end_point(2) - current_position(2);
        distance = sqrt(dx^2 + dy^2);
        angle_to_goal = atan2(dy, dx);

        % Calculate angular difference
        angle_diff = angle_to_goal - current_angle;

        % Normalize angle difference to be within [-pi, pi]
        angle_diff = atan2(sin(angle_diff), cos(angle_diff));

        % Set velocities
        velocityMsg.Linear.X = min(max_speed, distance);
        velocityMsg.Angular.Z = min(max_angular_speed, angle_diff);

        % Publish velocity message
        send(velocityPub, velocityMsg);

        % Break the loop if close to the end_point
        if distance < goal_threshold
            % Stop the robot
            velocityMsg.Linear.X = 0;
            velocityMsg.Angular.Z = 0;
            send(velocityPub, velocityMsg);
            break;
        end
        
        pause(0.1);
    end
end
