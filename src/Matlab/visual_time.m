directory='F:\Data\csv\';
usb_file=[directory,'usb.csv'];
build_file=[directory,'build.csv'];
locate_file=[directory,'locate.csv'];
fusion_file=[directory,'fusion.csv'];

usb_data=user_read_csv(usb_file);
build_data=user_read_csv(build_file);
locate_data=user_read_csv(locate_file);
fusion_data=user_read_csv(fusion_file);
mt=user_max_time(usb_data,build_data,locate_data,fusion_data);

fill_one_line(usb_data, 0.0, 0.1, 'r');
fill_one_line(build_data, 0.1, 0.2, 'g');
fill_one_line(locate_data, 0.2, 0.3, 'b');
fill_one_line(fusion_data, 0.3, 0.4, 'c');

axis equal
axis([0 mt, 0, 10]);
ax=gca;
ax.XGrid='on';