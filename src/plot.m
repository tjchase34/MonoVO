addpath(genpath('../'));

monovo_results = readtable('test.txt', 'Delimiter', ' ');
ground_truth = readtable('00.txt', 'Delimiter', ' ');

for i = 1:size(monovo_results, 1)
    T = monovo_results(i,:);
    r = table2array(T(:,1:12));
    R = [r(1), r(2), r(3), r(4);
         r(5), r(6), r(7), r(8);
         r(9), r(10), r(11), r(12);
         0,    0,    0,    1];
end

for i = 1:size(ground_truth, 1)
    T = ground_truth(i,:);
    r = table2array(T(:,1:12));
    R = [r(1), r(2), r(3), r(4);
         r(5), r(6), r(7), r(8);
         r(9), r(10), r(11), r(12);
         0,    0,    0,    1]
end