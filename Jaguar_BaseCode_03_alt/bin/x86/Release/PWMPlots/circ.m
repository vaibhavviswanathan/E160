%%


hf = figure(2);
set(hf,'PaperUnits','Points');
set(hf,'PaperPosition',[650,550,350,300]);

plot([1 3 5 3 1], [1 2 3 4 5], '*','LineWidth',2);
hold on;
plot(x,y);

xlabel 'x [m]' 
ylabel 'y [m]'

legend 'Waypoints' 'Actual Trajectory' 'Location' 'West'
grid on

title 'Circular Trajectory Tracking'

print -dpng -r500 './PWMPlots/CircularTrajectory.png'
