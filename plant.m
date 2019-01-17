function dx = plant(t,x,sys,u)
% external disturbances
v1 = 290.15; %ground?
v2 = 290.15; %ground?
v3 = 290.15;
v4 = 290.15;
v5 = 290.15;
v6 = 290.15;
v11 = 5*sin(2*pi*t/86400)+298.15; %sun
v22 = 5*sin(2*pi*t/86400)+298.15;
v33 = 5*sin(2*pi*t/86400)+298.15;
v44 = 5*sin(2*pi*t/86400)+298.15;

% internal disturbances
i1 = 0;
i2 = 0;
i3 = 0;
i4 = 0;
i5 = 0;
i6 = 0;
i11 = 0;
i22 = 0;
i33 = 0;
i44 = 0;


w = [i1;i2;i3;i4;i5;i6;i11;i22;
    i33;i44;v1;v2;v3;v4;v5;v6;
    v11;v22;v33;v44];% disturbances


dx = sys.A*x+sys.H*w+ sys.B*u;

% disp(u);
% null = input(['t = ' num2str(t)]);
end