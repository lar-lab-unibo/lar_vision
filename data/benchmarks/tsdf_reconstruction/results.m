
r_reference = [
    100
    50
    33
    25
    20
    16
    14
    12
    11
    10
    6
    5
];

r_duck = [
    1.10248
    1.10903
    1.10434
    1.10722
    1.12041
    1.12053
    1.13479
    1.13452
    1.15949
    1.17151
    1.16425
    1.32713
];

r_toycar = [
    3.17331
    3.20191
    3.57744
    3.51115
    3.64876
    3.44295
    3.44124
    3.52101
    3.53882
    3.54498
    3.74967
    3.83901    
];

r_vader = [
    2.52396
    2.55525
    2.62527
    2.67427
    2.78288
    2.63537
    2.79714
    2.7366
    2.6809
    2.94708
    2.97984
    3.30462
];

plot(r_reference,r_duck*3,'r', 'LineWidth',2);hold on;
plot(r_reference,r_vader*1.2,'g', 'LineWidth',2);hold on;
plot(r_reference,r_toycar,'b', 'LineWidth',2);hold on;
legend('DUCK','VADER','TOYCAR')
hold on;
xlabel('% of total frames (~150 frames)');
ylabel('MSE');
grid on;