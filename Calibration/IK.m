function q = IK(end_effector_position, links)

x = end_effector_position(1);
y = end_effector_position(2);
z = end_effector_position(3);

l1 = links(1);
l2 = links(2);

q1 = atan2(y,x);
q2 = z - l1;
q3 = sqrt(x^2 + y^2) - l2;

q = [q1 q2 q3];