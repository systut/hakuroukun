global Pos;
global theta;
global mx;
global goal;
global start;
global plus;
global main_goal;
global P1;
global P2;
global data;

plus = 1.0;
theta = 0.0;
theta_0 = 97.0;

theta = theta_0;
Pos = [0 0];
P1 = Pos;
P2 = Pos + [0 4];

fprintf("Current Pos : %f,%f,%f \n", Pos(1), Pos(2), theta);
current_pos = [Pos(1), Pos(2), theta];
start = [P1(1) P1(2)];
main_goal = [P2(1) P2(2)];
mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';
goal = GenerateGoal(start, plus, main_goal, mx, goal);

heading=CalcHeadingEval(mx,goal);

data = [current_pos , heading];
header = ["x", "y", "theta", "heading"];
WriteToCsv(data,header)
