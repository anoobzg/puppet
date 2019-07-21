function t = user_read_csv(name)
    A = importdata(name,',',1);
    data=A.data;
    t = data(:,2:3);
end
