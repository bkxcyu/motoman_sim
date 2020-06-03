function T = Trans_matrix(i,D_H)
% ���룺
%     D_H�����˲�����
%     i����i������
% �����
%     i-1��i�ı任����
% �Ҹо��������е��ñȽϷ���
%    by bxy
a=D_H(:,1);
alpha=D_H(:,2);
d=D_H(:,3);
theta=D_H(:,4);
T= [cos(theta(i))               -sin(theta(i))              0              a(i);
    sin(theta(i))*cos(alpha(i)) cos(theta(i))*cos(alpha(i)) -sin(alpha(i)) -d(i)*sin(alpha(i));
    sin(theta(i))*sin(alpha(i)) cos(theta(i))*sin(alpha(i)) cos(alpha(i))  d(i)*cos(alpha(i));
    0                           0                           0              1;
    ];
