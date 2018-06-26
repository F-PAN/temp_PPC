%% read PPC_data

clc;
clear all;

#load ../catkin_ws/src/PPC_data.dat
load ../PPC_data.dat
f=figure(1)
set(gca,'linewidth',1.5);
set(gca,'gridlinestyle','-');
plot(PPC_data(:,1),PPC_data(:,3),'r')
%plot(PPC_data(:,2),'r')
hold;
plot(PPC_data(:,1),PPC_data(:,2),'g')
%plot(PPC_data(:,3),'r')
title('force over time');
xlabel('time [s]')
#xlim([0 5]);
ylabel('force [N]')
grid on;

e=figure(2)
%set(gca,'linewidth',1.5);
%set(gca,'gridlinestyle','.');
scatter(PPC_data(:,1),PPC_data(:,7),'r')
hold on;
scatter(PPC_data(:,1),PPC_data(:,6),'g')
hold on;
scatter(PPC_data(:,1),PPC_data(:,5),'y')
%plot(PPC_data(:,1),PPC_data(:,6),'b')
title('performace bound and error ove rtime');
xlabel('error [N]')
%xlim([-10 0]);
ylabel('transformed error [N]')
grid on;

et=figure(3)
set(gca,'linewidth',1.5);
set(gca,'gridlinestyle','-');
plot(PPC_data(:,1),PPC_data(:,4),'color',[0.5 0.5 0.5])
%plot(PPC_data(:,4),'r')
hold on;
%plot(PPC_data(:,1),-1*PPC_data(:,5),'r')
%hold on;
plot(PPC_data(:,1),-1*PPC_data(:,10),'g')
hold on;
#scatter(PPC_data(:,1),-1*PPC_data(:,5),'g')
%title('performace bound and error over time');
hold on; 
line([0,20],[-1,-1],'color',[0.5 0.5 0.5]);
title('tracking error over time');
xlabel('time [s]')
#xlim([0 5]);
ylabel('error [N]')
#ylim([-3 6])
grid on;


v=figure(4)
set(gca,'linewidth',1.5);
set(gca,'gridlinestyle','-');
plot(PPC_data(:,1),PPC_data(:,8),'r')
hold on;
%plot(PPC_data(:,8),'g')
xlabel('time [s]')
%xlim([0 5]);
ylabel('velocity')

figure(5)
plot(PPC_data(:,9),PPC_data(:,6),'r')
hold on;
plot(PPC_data(:,9),PPC_data(:,7),'g')
