function [] = fill_one_line(A, y1, y2, k)
    m = size(A, 1);
    hold on
    for i = 1 : m
        xmin = A(i, 1);
        xmax = A(i, 2);
        x = zeros(4, 1);
        y = zeros(4, 1);
        x(1) = xmin;
        x(2) = xmax;
        x(3) = xmax;
        x(4) = xmin;
        y(1) = y1;
        y(2) = y1;
        y(3) = y2;
        y(4) = y2;
        fill(x, y, k);
    end
end