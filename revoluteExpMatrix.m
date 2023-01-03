function T = revoluteExpMatrix(se3mat,theta)

R = eye(3) + sin(theta) * se3mat(1: 3, 1: 3) + (1 - cos(theta)) * se3mat(1: 3, 1: 3) * se3mat(1: 3, 1: 3);

T = [R, (eye(3) * theta + (1 - cos(theta)) * se3mat(1: 3, 1: 3) ...
          + (theta - sin(theta)) * se3mat(1: 3, 1: 3) * se3mat(1: 3, 1: 3)) ...
            * se3mat(1: 3, 4) / theta;
         0, 0, 0, 1];
end