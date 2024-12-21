clc;
clear;
close all;
test = readmatrix('formatted_data2.csv');
angle= test(:,1);
BENDload= test(:,2);
COMPload= test(:,3);
figure(1)
subplot(1,3,1)
plot(angle,BENDload*.45);
xlabel("Angle [deg]")
ylabel("Moment [Nm]")
fontsize(16,"points")
subplot(1,3,2)
plot(angle,COMPload);
xlabel("Angle [deg]")
ylabel("Axial Load[kg]")
fontsize(16,"points")
subplot(1,3,3)
plot(angle,BENDload)
xlabel("Angle [deg]")
ylabel("Bending Load[kg]")
fontsize(16,"points")
saveas(gcf,'DEMO_test.png');
figure(2)
plot(angle,BENDload*.45/100,'LineWidth',2,'Color',"black");
hold on
inflection=find(angle==2)
IA_M=[BENDload(1)*.45/100;BENDload(inflection)*.45/100;BENDload(end)*.45/100];
IA_a=[angle(1);angle(inflection);angle(end)];
K1=(IA_M(2)-IA_M(1))/(IA_a(2)-IA_a(1))
K2=(IA_M(3)-IA_M(2))/(IA_a(3)-IA_a(2))
IA=K2-K1
txt1 = ['K1=',num2str(K1)];
text(-8,0.25,txt1)
txt2 = ['K2=',num2str(K2)];
text(1,4,txt2)
if (IA >0)
   txt3 = ['IA = K2-K1 =',num2str(IA),'>0'];
   text(-6,2,txt3,'Color',"green")
   plot(IA_a,IA_M,'LineWidth',2,'Color',"green")
elseif (IA<0)
   txt3 = ['IA = K2-K1 =',num2str(IA),'<0'];
   text(-6,2,txt3,'Color',"red")
   plot(IA_a,IA_M,'LineWidth',2,'Color',"red")
end
xlim([-10 7])
xlabel("Angle [deg]")
ylabel("Moment(Normalized) [Nm/kg]")
fontsize(16,"points")
saveas(gcf,'IAcharacterization.png');
