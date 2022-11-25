close all;
clear all;

%% 데이터 불러오기
dataHist = dlmread("log.txt");
t = dataHist(:,1);
q1 = dataHist(:,2);
q2 = dataHist(:,3);

%% 데이터 표기
figure();
plot(t, q1);
hold on;
plot(t, q2);
grid on;

%% 차트에 제목 및 축 레이블 추가
title("Simulation result");
xlabel("time[s]");
ylabel("angle[rads]");

legend("q_1", "q_2");