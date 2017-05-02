function [raw,MAF,IIR,FIR] = client(port,collect_size)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port,'BaudRate', 230400,'Timeout',20); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

% has_quit = false;
% menu loop
% while ~has_quit
fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
% display the menu options; this list will grow
fprintf('Press r for data\r\n');
% read the user's choice
selection = input('\nENTER COMMAND: ', 's');

% send the command to the PIC32
fprintf(mySerial,'%c\n',selection);

raw = zeros(collect_size,1);
MAF = zeros(collect_size,1);
IIR = zeros(collect_size,1);
FIR = zeros(collect_size,1);

% take the appropriate action
for i = 1:1:collect_size
    aa = fscanf(mySerial,'%c\r\n');
    aaa = textscan(aa,'%f,%f,%f,%f,%f');
    thedata = cell2mat(aaa);
    check_size = size(thedata);
    if check_size(1,1) == 0
        thedata=zeros(1,5);
    end

    raw(i,1) = thedata(1,2);
    MAF(i,1) = thedata(1,3);
    IIR(i,1) = thedata(1,4);
    FIR(i,1) = thedata(1,5);
end
%     data = fscanf(mySerial,'%c\r\n');
%     data2 = fscanf(mySerial,'%c\r\n');
%     data3 = fscanf(mySerial,'%c\r\n');
% end

% plot the fft of the raw data
rawY = fft(raw);
rawP2 = abs(rawY/collect_size);
rawP1 = rawP2(1:collect_size/2+1);
rawP1(2:end-1) = 2*rawP1(2:end-1);
rawff = 100*(0:(collect_size/2))/collect_size;
figure;
loglog(rawff,rawP1,'r') 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
hold on;

MAFY = fft(MAF);
MAFP2 = abs(MAFY/collect_size);
MAFP1 = MAFP2(1:collect_size/2+1);
MAFP1(2:end-1) = 2*MAFP1(2:end-1);
MAFff = 100*(0:(collect_size/2))/collect_size;
loglog(MAFff,MAFP1,'b')

IIRY = fft(IIR);
IIRP2 = abs(IIRY/collect_size);
IIRP1 = IIRP2(1:collect_size/2+1);
IIRP1(2:end-1) = 2*IIRP1(2:end-1);
IIRff = 100*(0:(collect_size/2))/collect_size;
loglog(IIRff,IIRP1,'g')

FIRY = fft(FIR);
FIRP2 = abs(FIRY/collect_size);
FIRP1 = FIRP2(1:collect_size/2+1);
FIRP1(2:end-1) = 2*FIRP1(2:end-1);
FIRff = 100*(0:(collect_size/2))/collect_size;
loglog(FIRff,FIRP1,'y')
legend('raw','MAF','IIR','FIR')



% plot the data
figure;
plot(raw,'r');
hold on;
plot(MAF,'b');
plot(IIR,'g');
plot(FIR,'y');
legend('raw','MAF','IIR','FIR')

end
