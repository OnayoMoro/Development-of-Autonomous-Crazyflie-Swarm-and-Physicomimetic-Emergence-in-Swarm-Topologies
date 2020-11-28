goal = 10;
Iterations = 50;
current_val = 0;
op = [current_val];
data(1:1,1:Iterations) = goal;

Kp = 0.8;
Ki = 0.01;
Kd = 0.9;
t = 0.1;

e = 0;
prev_e = 0;
Integral = 0;
Derivative = 0;

for i = 1:Iterations-1
    
    e = goal - current_val;
    
    Integral = Integral + (t * e);   
    Derivative = (e - prev_e) / t;
    prev_e = 0;
    prev_e = e; e = 0;
    
    output = (Kp * e) + (Ki * Integral) + (Kd * Derivative);
    
    current_val = current_val + output * t;
    op = [op current_val];
    
end

hold on;
plot (op,'DisplayName','PID Output');
plot (data,'DisplayName','Goal');
hold off;