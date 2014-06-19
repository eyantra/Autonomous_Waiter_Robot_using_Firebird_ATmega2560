
clear;
%fclose(instrfind);
%s1 = input('Enter the x co-ordinate : ');
%s2 = input('Enter the y co-ordinate : ');

ser = serial('COM31');% (put whatever port number corresponds to the XBee)
fopen(ser);
set(ser,'Timeout',600);

%fprintf(ser, s1);
%fprintf(ser, s2);

vid=videoinput('winvideo',2);	%2 corresponds to the camera number of the cam used
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb');

while(1)	%continously read the value at the port
    
s=fread(ser,1);
if(s==1)   %bot stops on junction and sends "1" via the ZigBee

	data = getsnapshot(vid);	%take an image when the arm of the bot has moved left
	break;

end
end

rgb1=data(:,:,1);
rgb2=data(:,:,2);
rgb3=data(:,:,3);

count_r = 0;	%the amount of red component in the image, and so on
count_b = 0;
count_g = 0;
total_count=0;

for i=1:160		%can change the range according to the resolution of the camera used
	for j=1:120	
		comp_r = rgb1(j,i);
		comp_g = rgb2(j,i);
		comp_b = rgb3(j,i);
		total_count = total_count + 1;
		if((comp_r > 150) && (comp_g<100) && (comp_b < 100)) count_r = count_r + 1; end;
		if((comp_b > 150) && (comp_g<100) && (comp_r < 100)) count_b = count_b + 1; end;
		if((comp_g > 150) && (comp_r<100) && (comp_b < 100)) count_g = count_g + 1; end;
	end
end

%compare the %age colour component corresponding to the colour of the object
perc_r = (count_r*100)/total_count;
perc_b = (count_b*100)/total_count;
perc_g = (count_g*100)/total_count;

%compare the required minimum %age to qualify for the object to be present
%if(perc >= 5) 
fprintf(ser, '1'); 
%end
%if(perc < 5) 
%fprintf(ser, '0'); 
%end