%clear A D datagram Data Oldstat pitch integral Err ias i K it throttle elev pitch_trim
%u2 = udp('127.0.0.1','LocalPort',49003);
%u2.EnablePortSharing = 'on';
%fopen(u2);
n=3;
No_command=single(-999);
%u = udp('127.0.0.1',49000);
%fopen(u);
header=uint8([68 65 84 65 0]);
gear_brake_index=uint8([14 0 0 0]);
gear_brake_cmd=[No_command 0 No_command No_command 0 No_command No_command No_command];
throttle_cmd_index=uint8([25 0 0 0]);
throttle=0;
%throttle_cmd=[throttle throttle No_command No_command No_command No_command No_command No_command];
primary_ctr_index=uint8([11 0 0 0]);
sec_ctr_index=uint8([13 0 0 0]);
datagram=serialize(header,gear_brake_index,gear_brake_cmd);
fwrite(u,datagram);
pitchRef=0;
%altRef=1000;
elev=0;
it=1;
pitch_trim=0;
IASRef=200;
Kairspeed=[0.5 0 1.5];
Kpitch=2.5;
Kpitchrate=0.3;
Integral=[0 0]';
Oldstat=[0 0;0 0];
pitch=[0 0];
t(1)=0;
%p=plot(pitch);
%p.YDataSource = 'pitch';
while(true)
    tic
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
ias(it)=Data(1,1);
%alt(it)=Data(4,6);%-Data(3,1);
pitch(it)=Data(3,1);
Err=[pitchRef IASRef]'-[pitch(it) ias(it)]';
D(1:2,it)=[[pitch(it) ias(it)]' Oldstat]*[3 -4 1]';
Oldstat(1:2,2)=Oldstat(1:2,1);
Oldstat(1:2,1)=[pitch(it) ias(it)];
Integral=Integral+Err;
throttle=throttle+Kairspeed*[Err(2);Integral(2);D(2,it)];
elev = Kpitchrate*(Kpitch*(pitchRef - pitch(it)) - D(1,it));
if throttle>1
    throttle=1;
end
if throttle<0
    throttle=0;
end
if elev>1
    elev=1;
end
if elev<-1
    elev=-1;
end
throttle_cmd=[throttle throttle No_command No_command No_command No_command No_command No_command];
primary_ctr_cmd=[elev No_command No_command No_command No_command No_command No_command No_command];
sec_ctr_cmd=[0 No_command No_command No_command No_command No_command No_command No_command];
datagram=serialize(header,throttle_cmd_index,throttle_cmd,primary_ctr_index,primary_ctr_cmd,sec_ctr_index,sec_ctr_cmd);
fwrite(u,datagram);
T(it)=toc;
if T(it)>0.1
    it=it+1;
    %t(it)=t(it-1)+T(it);
end
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