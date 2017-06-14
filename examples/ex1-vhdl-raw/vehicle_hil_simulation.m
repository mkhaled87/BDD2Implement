%
% robot.m
%
% created on: 15.12.2016
%     author: M. Khaled
%
% You need to run ./ex3 first to generate the VHDL code and synthesize the HW inside the FPGA
% You need to have an VPGA running the required Linux-sys/BDD-hw and connected to the PC
%

function robot
clear set
close all

%% Initialization
x0=[0.4 0.4 0];
tau = 0.3;
SIM_STEPS   = Inf;

x1_first = 0;
x1_eta = 0.2;
x2_first = 0;
x2_eta = 0.2;
x3_first = -3.4;
x3_eta = 0.2;
x1_nbits = 6;
x2_nbits = 6;
x3_nbits = 6;

u1_first = -0.9;
u2_first = -0.9;
u1_eta = 0.3;
u2_eta = 0.3;
u1_nbits = 3;
u2_nbits = 3;

%% some resources for visualizing results
resources = 'scots-files-vehicle/';
colors=get(groot,'DefaultAxesColorOrder');

controller = SymbolicSet([resources 'vehicle_controller.bdd'],'projection',[1 2 3]);
states     = SymbolicSet([resources 'vehicle_ss.bdd'], 'projection',[1 2]);
target     = SymbolicSet([resources 'vehicle_ts.bdd'], 'projection',[1 2]);
obstacles  = SymbolicSet([resources 'vehicle_obst.bdd'], 'projection',[1 2]);

% display the symbolic set containig states/target/obstacles
plotCells(states,'facecolor','none','edgec',[0.8 0.8 0.8],'linew',.1)
plotCells(obstacles,'facecolor',colors(1,:)*0.5+0.5,'edgec',colors(1,:),'linew',.1)
plotCells(target,'facecolor',colors(2,:)*0.5+0.5,'edgec',colors(2,:),'linew',.1)

hold on

%% opening the serial port and testing the connection
comport = terminal_open();
terminal_write(comport, 'ping');
out = terminal_read(comport);
if(strcmp(out,'pong') == 0)
    termina_close(comport);
    error('Check whether the HIL program is running in the hardware !')
end


%% HIL Simulation
y=x0;
i=0;
while(i<SIM_STEPS)
  if (target.isElement(y(end,:)))
    break;
  end 
  
  x1 = round((y(end,1)-x1_first)/x1_eta);
  x2 = round((y(end,2)-x2_first)/x2_eta);
  x3 = round((y(end,3)-x3_first)/x3_eta);
  
  x3 = bitshift(x3, x1_nbits+x2_nbits);
  x2 = bitshift(x2, x1_nbits);
  x = bitor(x1,bitor(x2,x3));
    
  terminal_write(comport,num2str(x));
  u = str2double(terminal_read(comport));
  
  safe_mask = bitshift(1, u1_nbits + u2_nbits)-1;
  u     = bitand(u, safe_mask);
  
  
  mask1 = bitshift(1, u1_nbits) -1;  
  mask2 = bitshift(mask1, u2_nbits);  
  u1    = bitand(u, mask1);
  u2    = bitand(u, mask2);
  u2    = bitshift(u2, -u1_nbits);  
  
  u1 = (u1*u1_eta)+u1_first;
  u2 = (u2*u2_eta)+u2_first;
  
  
  
  %u=controller.getInputs(y(end,:));
  [t x]=ode45(@vehicle_ode,[0 tau], y(end,:),[],[u1 u2]);

  y=[y; x(end,:)];
  i = i+1;
end


%% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)

box on
axis([-.5 10.5 -.5 10.5])


termina_close(comport);

end

function termina_close(port)
    fclose(port)
    delete(port)
    clear port
end

function port = terminal_open()
    COM_PORT = '/dev/ttyACM0';
    BAUD_RATE = 115200;
    try        
        port = serial(COM_PORT, 'Baudrate', BAUD_RATE, 'Parity', 'none', 'DataBits', 8, 'StopBits', 1, 'Terminator', 'LF', 'Timeout', 1);
        fopen(port);
    catch
        termina_close(port);
        error('Failed to open the terminal');
    end
end

function terminal_write(port, str)
    fprintf(port, [str '\n']);
    java.lang.Thread.sleep(length([str '\n']));
    echo = fgets(port);
    echo(end-1:end)=[]; % removing the \n and \r
    if(strcmp(str, echo) == 0)
        termina_close(port);
        error('Failed to write to terminal');
    end
end
function out = terminal_read(port)
    out = fgets(port);
    out(end-1:end)=[]; % removing the \n and \r
end


function dxdt = vehicle_ode(t,x,u)

  dxdt = zeros(3,1);
  c=atan(tan(u(2)/2));

  dxdt(1)=u(1)*cos(c+x(3))/cos(c);
  dxdt(2)=u(1)*sin(c+x(3))/cos(c);
  dxdt(3)=u(1)*tan(u(2));


end
