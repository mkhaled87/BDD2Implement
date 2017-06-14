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
x0=[1 4];
tau = 1;
SIM_STEPS   = 100;
x1_first = 0;
x1_eta = 1;
x2_first = 0;
x2_eta = 1;
x1_nbits = 4;
x2_nbits = 4;
u1_nbits = 2;
u2_nbits = 2;

%% some resources for visualizing results
resources = 'scots-files-robot/';
colors=get(groot,'DefaultAxesColorOrder');

controller = SymbolicSet([resources 'robot_controller.bdd'],'projection',[1 2]);
states     = SymbolicSet([resources 'robot_ss.bdd']);
target     = SymbolicSet([resources 'robot_ts.bdd']);
obstacles  = SymbolicSet([resources 'robot_obst.bdd']);

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


%% Simulation
y=x0;
i=0;
while(i<SIM_STEPS)

  x1 = round((y(end,1)-x1_first)/x1_eta);
  x2 = round((y(end,2)-x2_first)/x2_eta);
  x2 = bitshift(x2, x1_nbits);
  x = bitor(x1,x2);
    
  terminal_write(comport,num2str(x));
  u = str2double(terminal_read(comport));
  
  safe_mask = bitshift(1, u1_nbits + u2_nbits)-1;
  u     = bitand(u, safe_mask);
  
  
  mask1 = bitshift(1, u1_nbits) -1;  
  mask2 = bitshift(mask1, u2_nbits);  
  u1    = bitand(u, mask1);
  u2    = bitand(u, mask2);
  u2    = bitshift(u2, -u1_nbits);  
  
  u1 = u1-1;
  u2 = u2-1;
  
  
  
  %u=controller.getInputs(y(end,:));
  [t x]=ode45(@robot_ode,[0 tau], y(end,:),[],[u1 u2]);

  y=[y; x(end,:)];
  i = i+1;
end


%% plot initial state  and trajectory
plot(y(:,1),y(:,2),'k.-')
plot(y(1,1),y(1,2),'.','color',colors(5,:),'markersize',20)

box on
axis([-.5 15.5 -.5 15.5])


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


function dxdt = robot_ode(t,x,u)
  dxdt = zeros(2,1);
  dxdt(1)=u(1);
  dxdt(2)=u(2);
end
