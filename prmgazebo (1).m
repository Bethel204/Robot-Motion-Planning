% Define an environment with obstacles
env = robotics.BinaryOccupancyGrid(10, 10, 10);

% Define obstacles as circles
obstacles = [3, 3, 1; 6, 6, 2; 8, 2, 1.5];  % [x, y, radius]

% Insert obstacles into the occupancy grid
for i = 1:size(obstacles, 1)
    radius = obstacles(i, 3);
    centerX = obstacles(i, 1);
    centerY = obstacles(i, 2);
    for x = -radius:0.1:radius
        for y = -radius:0.1:radius
            if x^2 + y^2 <= radius^2
                setOccupancy(env, [centerX + x, centerY + y], 1);
            end
        end
    end
end

% Create a PRM planner
prm = robotics.PRM;
prm.Map = env;
prm.NumNodes = 50;       % Number of milestones
prm.ConnectionDistance = 5;  % Maximum distance to connect nodes

% Define start and goal positions
start = [1, 1];
goal = [9, 9];

% Find a path is a function that utilizes robotics.PRM
path = findpath(prm, start, goal);

% Visualize the result as a graph
show(prm);
hold on;
plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal(1), goal(2), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
hold off;



rosinit('http://localhost:11311');
velPub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
velMsg = rosmessage(velPub);
function moveToGoal(xGoal, yGoal, velPub, velMsg)
    subOdom = rossubscriber('/odom', 'nav_msgs/Odometry');
    [odomMsg, ~] = receive(subOdom, 3);
    position = odomMsg.Pose.Pose.Position;
    orientation = odomMsg.Pose.Pose.Orientation;
    angles = quat2eul([orientation.W orientation.X orientation.Y orientation.Z]);
    theta = angles(1);
    goalDistance = sqrt((xGoal - position.X)^2 + (yGoal - position.Y)^2);
    
    while goalDistance > 0.1
        linearSpeed = 0.2 * goalDistance;
        desiredAngle = atan2(yGoal - position.Y, xGoal - position.X);
        angularSpeed = 1.0 * (desiredAngle - theta);
        
        velMsg.Linear.X = linearSpeed;
        velMsg.Angular.Z = angularSpeed;
        
        send(velPub, velMsg);
        
        odomMsg = receive(subOdom, 3);
        position = odomMsg.Pose.Pose.Position;
        orientation = odomMsg.Pose.Pose.Orientation;
        angles = quat2eul([orientation.W orientation.X orientation.Y orientation.Z]);
        theta = angles(1);
        goalDistance = sqrt((xGoal - position.X)^2 + (yGoal - position.Y)^2);
    end
    
    velMsg.Linear.X = 0;
    velMsg.Angular.Z = 0;
    send(velPub, velMsg);
end

for i = 1:size(path, 1)
    moveToGoal(path(i, 1), path(i, 2), velPub, velMsg);
end

rosshutdown;
