global start;       %ロボットの初期位置
global goal;
global main_goal;   %終了ポイント
global mx;          %ロボットの初期状態 [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
global plus;
theta = 0;
Pos = [0 0];
P1 = Pos;
P2 = P1+[0 4];
P3 = P1+[-3 4];
P4 = P1+[-3 0];

P = [P1; P2; P3; P4; P1];

start=[P1(1) P1(2)];
plus = 1.5;
main_goal = [P2(1) P2(2)]';
mx = [Pos(1) Pos(2) deg2rad(theta) 0 0]';

goal = GenerateGoal(start, plus, main_goal, mx, goal);
