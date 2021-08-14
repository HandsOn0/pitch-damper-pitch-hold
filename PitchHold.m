%clear A D datagram Data Oldstat pitch integral Err ias i K it throttle elev pitch_trim
u2 = udp('127.0.0.1','LocalPort',49003);
%u2.EnablePortSharing = 'on';
fopen(u2);
n=1;
No_command=single(-999);
u = udp('127.0.0.1',49000);
fopen(u);
header=uint8([68 65 84 65 0]);
gear_brake_index=uint8([14 0 0 0]);
gear_brake_cmd=[No_command 0 No_command No_command 0 No_command No_command No_command];
throttle_cmd_index=uint8([25 0 0 0]);
throttle=1;
throttle_cmd=[throttle throttle No_command No_command No_command No_command No_command No_command];
primary_ctr_index=uint8([11 0 0 0]);
sec_ctr_index=uint8([13 0 0 0]);
datagram=serialize(header,gear_brake_index,gear_brake_cmd,throttle_cmd_index,throttle_cmd);
fwrite(u,datagram);
pitchRef=5;
elev=0;
K=[0.224 0 0];
Integral=0;
Oldpitch=[0 0];
it=1;
while(true)
   l=0;
   while(l~=5+n*36)
 A=fread(u2,5+n*9*4);
   [l dummy]=size(A);
   end
Data=zeros(n,8);
for i=1:n
    for j=2:9  
Data(i,j-1)=typecast(uint8(A(6+(i-1)*36+(j-1)*4:(i-1)*36+6+(j-1)*4+3)),'single');
    end
end
pitch(it)=Data(1,1);
%% PID Controller Code %%
% Calculate the error
Err=pitchRef-pitch(it);
% Calculate the pitch angle derivative
D(it)=[pitch(it) Oldpitch]*[3 -4 1]';
%Update the the previous values 
Oldpitch(2)=Oldpitch(1);
Oldstat(1)=pitch(it);
% Calculate the integral
Integral=Integral+Err;
%Calculate the output
elev=K*[Err Integral -D(it)]';

% add saturation to elevators deflection
if elev>1
    elev=1;
end
if elev<-1
    elev=-1;
end
%END PID
it=it+1;
primary_ctr_cmd=[elev No_command No_command No_command No_command No_command No_command No_command];
datagram=serialize(header,primary_ctr_index,primary_ctr_cmd);
fwrite(u,datagram);
end
function output=serialize(varargin)
[dummy n]=size(varargin);
id=1;
for i=1:n
w=typecast(varargin{i},'uint8');
[x y]=size(w);
output(id:id+y-1)=w;
id=id+y;
end
end