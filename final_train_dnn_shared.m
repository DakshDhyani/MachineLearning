clc
clear
         
%Loading the all the files for data
load df_40_1.dat
load dfcc20_2_2.dat
load dfcc20_2_1.dat
load csub7_3.dat
load adc20_2_12_1.dat
load adc20_2_34_3.dat
load adc10_2_12_1.dat  %this file is only for aircraft status.

%checking aircraft status for finding in air time.
aircraft_status = adc10_2_12_1(:,74);

%To find the first and the last occurence of 1 (for in air data).
D  = diff([0;aircraft_status;0]);
S = find(D>0);
E = find(D<0)-1;

%storing the index to start for in air data(+500 and -500 is done to get
%the proper in air data.
refer_index_start = S(1)+500;
refer_index_end = E(1)-500;

%refered time (on the basis on which , we will take the nearest time for
%other files to calculate in air data).
refered_time_start = adc10_2_12_1(refer_index_start,2);
refered_time_end = adc10_2_12_1(refer_index_end,2);

%Take the time columns for all the file to take in air time.
time_df_40_1 = df_40_1(:,2);
time_dfcc20_2_2 = dfcc20_2_2(:,2);
time_dfcc20_2_1 = dfcc20_2_1(:,2);
time_csub7_3 = csub7_3(:,2);
time_adc20_2_12_1 = adc20_2_12_1(:,2);
time_adc20_2_34_3 = adc20_2_34_3(:,2);


%find the next nearest time in rest of the files for 
%the in air data.
near_time_index_start_df_40_1 = find( (time_df_40_1(:,1)) > refered_time_start,1);
near_time_index_end_df_40_1 = find( (time_df_40_1(:,1)) > refered_time_end,1);

near_time_index_start_dfcc20_2_2 = find( (time_dfcc20_2_2(:,1)) > refered_time_start,1);
near_time_index_end_dfcc20_2_2 = find( (time_dfcc20_2_2(:,1)) > refered_time_end,1);

near_time_index_start_dfcc20_2_1 = find( (time_dfcc20_2_1(:,1)) > refered_time_start,1);
near_time_index_end_dfcc20_2_1 = find( (time_dfcc20_2_1(:,1)) > refered_time_end,1);

near_time_index_start_csub7_3 = find( (time_csub7_3(:,1)) > refered_time_start,1);
near_time_index_end_csub7_3 = find( (time_csub7_3(:,1)) > refered_time_end,1);

near_time_index_start_adc20_2_12_1 = find( (time_adc20_2_12_1(:,1)) > refered_time_start,1);
near_time_index_end_adc20_2_12_1 = find( (time_adc20_2_12_1(:,1)) > refered_time_end,1);

near_time_index_start_adc20_2_34_3 = find( (time_adc20_2_34_3(:,1)) > refered_time_start,1);
near_time_index_end_adc20_2_34_3 = find( (time_adc20_2_34_3(:,1)) > refered_time_end,1);

%Extracting all the states to have the parameters after transformation.
lat_acc = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,6);
normal_acc = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,5);

roll_altitude = dfcc20_2_2(near_time_index_start_dfcc20_2_2:near_time_index_end_dfcc20_2_2,7);
pitch_altitude = dfcc20_2_2(near_time_index_start_dfcc20_2_2:near_time_index_end_dfcc20_2_2,9);
yaw_altitude = dfcc20_2_2(near_time_index_start_dfcc20_2_2:near_time_index_end_dfcc20_2_2,6);

roll_rate = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,7);
pitch_rate = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,8);
yaw_rate = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,9);

velocity_x = dfcc20_2_1(near_time_index_start_dfcc20_2_1:near_time_index_end_dfcc20_2_1,11);
velocity_y = dfcc20_2_1(near_time_index_start_dfcc20_2_1:near_time_index_end_dfcc20_2_1,6);
velocity_z = dfcc20_2_1(near_time_index_start_dfcc20_2_1:near_time_index_end_dfcc20_2_1,8);

H = csub7_3(near_time_index_start_csub7_3:near_time_index_end_csub7_3,59);

lie_ram_pos_cmd = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,24);
rie_ram_pos_cmd = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,26);
rudder_ram_pos_cmd = df_40_1(near_time_index_start_df_40_1:near_time_index_end_df_40_1,28);

total_pressure_mach = adc20_2_34_3(near_time_index_start_adc20_2_34_3:near_time_index_end_adc20_2_34_3 ,8);
static_pressure_mach = adc20_2_34_3(near_time_index_start_adc20_2_34_3:near_time_index_end_adc20_2_34_3 ,9);

%input for the values to be predicted
aoa_fbk = adc20_2_12_1(near_time_index_start_adc20_2_12_1:near_time_index_end_adc20_2_12_1,9);
ssa_fbk = adc20_2_12_1(near_time_index_start_adc20_2_12_1:near_time_index_end_adc20_2_12_1,12);
mach_no = csub7_3(near_time_index_start_csub7_3:near_time_index_end_csub7_3,60);



% if this is odd then dont take the last entry for expanded 
%otherwise take it.
to_check = size(lat_acc,1);

% transformation of phi(roll altitude) and theta(pitch altitude)
sined_roll = sin(roll_altitude);
cosined_roll = cos(roll_altitude);
sined_pitch = sin(pitch_altitude);
cosined_pitch = cos(pitch_altitude);
sined_yaw = sin(yaw_altitude);
cosined_yaw = cos(yaw_altitude);

%Transformation for Velocity x,y and z to body frame.
n = size(velocity_x,1);
velocity_x_body = ones(n,1);
velocity_y_body = ones(n,1);
velocity_z_body = ones(n,1);

for i =1 : n
    velocity_x_body(i,1) = ( (cosined_pitch(i,1)*cosined_yaw(i,1)*velocity_x(i,1)) + (cosined_pitch(i,1)*sined_yaw(i,1)*velocity_y(i,1)) - (sined_pitch(i,1)*velocity_z(i,1)));
    velocity_y_body(i,1) = ( ((-cosined_roll(i,1)*sined_yaw(i,1) + cosined_yaw(i,1)*sined_pitch(i,1)*sined_roll(i,1))*velocity_x(i,1) ) + ((cosined_roll(i,1)*cosined_yaw(i,1) + sined_yaw(i,1)*sined_pitch(i,1)*sined_roll(i,1))*velocity_y(i,1)) + (sined_roll(i,1) * cosined_pitch(i,1))*velocity_z(i,1));
    velocity_z_body(i,1) = ( ((sined_roll(i,1)*sined_yaw(i,1) + cosined_roll(i,1)*sined_pitch(i,1)*cosined_yaw(i,1))*velocity_x(i,1)) +((-sined_roll(i,1)*cosined_yaw(i,1) + cosined_roll(i,1)*sined_pitch(i,1)*sined_yaw(i,1))*velocity_y(i,1)) + (cosined_roll(i,1)*cosined_pitch(i,1)*velocity_z(i,1)));
end

%now converting the data from 20hz to 40hz for
%transformed theta,phi,and the three velocities

h= @data_expand; % function declaration for usage
expanded_sined_roll = data_expand(sined_roll,to_check);
expanded_cosined_roll = data_expand(cosined_roll,to_check);
expanded_sined_pitch = data_expand(sined_pitch,to_check);
expanded_cosined_pitch = data_expand(cosined_pitch,to_check);

expanded_velocity_x = data_expand(velocity_x_body,to_check);
expanded_velocity_y = data_expand(velocity_y_body,to_check);
expanded_velocity_z = data_expand(velocity_z_body,to_check);
expanded_aoa_fbk = data_expand(aoa_fbk,to_check);
expanded_ssa_fbk = data_expand(ssa_fbk,to_check);
%now converting the 10hz to 40hz for H
expand_10_2_20_H = data_expand(H,1); %passed 1 here so that it would not take 2 extra entry in case of even.
expanded_H = data_expand(expand_10_2_20_H,to_check);

expand_mach_no = data_expand(mach_no,1);
expanded_mach_no = data_expand(expand_mach_no,to_check);

expanded_total_pressure_mach = data_expand(total_pressure_mach,to_check);
expanded_static_pressure_mach = data_expand(static_pressure_mach,to_check);


%Output array
arr_output = [expanded_aoa_fbk,expanded_ssa_fbk,expanded_mach_no];


%for the estimation of alpha i.e. angle of attack
arr_in_alpha = [pitch_rate,normal_acc, expanded_velocity_z,rie_ram_pos_cmd,rudder_ram_pos_cmd,lie_ram_pos_cmd,expanded_velocity_y];
arr_in_alpha = normalize(arr_in_alpha);
arr_out_alpha = expanded_aoa_fbk;

%for estimation of beta i.e. angle of sideship
arr_in_beta= [lat_acc ,roll_rate,yaw_rate,expanded_velocity_x,expanded_velocity_y,expanded_H,lie_ram_pos_cmd,rie_ram_pos_cmd,rudder_ram_pos_cmd];
arr_in_beta = normalize(arr_in_beta);
arr_out_beta = expanded_ssa_fbk;

%for estimation of M i.e. mach number
arr_in_mach = [expanded_H,expanded_velocity_y,expanded_velocity_x,rie_ram_pos_cmd,lie_ram_pos_cmd,expanded_static_pressure_mach ];
arr_in_mach = normalize(arr_in_mach);
arr_out_mach = expanded_mach_no;

% For the training of alpha and beta
input_size_alpha = 7;
outputSize =1; 

input_size_beta = 9;

%For mach number
input_size_mach = 6;


numHiddenUnits = 1024;
dropoutRate = 0.2;

layers = [ sequenceInputLayer(input_size_alpha)
            lstmLayer(numHiddenUnits,'OutputMode','sequence','Name','lstm_1')
            dropoutLayer(dropoutRate,'Name','dropout_1')
            lstmLayer(numHiddenUnits,'OutputMode','sequence','Name','lstm_2')
            dropoutLayer(dropoutRate,'Name','dropout_2')
            fullyConnectedLayer(outputSize,'Name','output')
            regressionLayer('Name','output')];
        
        
layers_beta = [ sequenceInputLayer(input_size_beta)
            lstmLayer(numHiddenUnits,'OutputMode','sequence','Name','lstm_1')
            dropoutLayer(dropoutRate,'Name','dropout_1')
            lstmLayer(numHiddenUnits,'OutputMode','sequence','Name','lstm_2')
            dropoutLayer(dropoutRate,'Name','dropout_2')
            fullyConnectedLayer(outputSize,'Name','output')
            regressionLayer('Name','output')];        


layers_mach = [ sequenceInputLayer(input_size_mach)
            lstmLayer(numHiddenUnits,'OutputMode','sequence','Name','lstm_1')
            dropoutLayer(dropoutRate,'Name','dropout_1')
            lstmLayer(numHiddenUnits,'OutputMode','sequence','Name','lstm_2')
            dropoutLayer(dropoutRate,'Name','dropout_2')
            fullyConnectedLayer(outputSize,'Name','output')
            regressionLayer('Name','output')];



options = trainingOptions('adam', ...
    'InitialLearnRate',0.001, ...
    'MaxEpochs',200, ...
    'MiniBatchSize',16, ...
    'Plots','training-progress');


%1st Dnn for the alpha i.e. angle of attack
net = trainNetwork(arr_in_alpha',arr_out_alpha',layers,options);
save('network_dnn_alpha_finalsubmit.mat','net');

%2nd Dnn for the beta i.e. angle of sideship
net1 = trainNetwork(arr_in_beta',arr_out_beta',layers_beta,options);
save('network_dnn_beta_finalsubmit.mat','net1');

%3rd Dnn for the M i.e. Mach number
net2 = trainNetwork(arr_in_mach',arr_out_mach',layers_mach,options);
save('network_dnn_mach_finalsubmit.mat','net2');

%Function to make all the parameters at same level i.e. all at 40 hz frequency.
function arr = data_expand(array,to_check)
    % if this is odd then dont take the last entry for expanded 
    %otherwise take it.

    n= size(array);
    no_of_rows = n(1);
    new_no_of_row = no_of_rows*2;
    if mod(to_check,2) == 0
        %take last entry too if even
        arr = ones(new_no_of_row,1);
    else
        %dont take last entry for odd
        arr = ones(new_no_of_row-1,1);
    end
    y = 1;
    m=1;
    x= 1;

    while(x<no_of_rows)
        arr(m,y) = array(x,y);
        m= m+2;
        x = x+1;
        arr(m,y) = array(x,y);
        m=m-1;
        arr(m,y) = (arr(m+1,y) + arr(m-1,y))/2;
        m=m+2;
        if(x == no_of_rows)
            break
        end
        arr(m,y) = (array(x+1,y) + arr(m-1,y))/2;
        m=m+1;
        x=x+1;
    
    end
    arr(new_no_of_row-1,y) = array(no_of_rows,y);
    if mod(to_check,2) == 0
        %retain previous value in case of even
        arr(new_no_of_row,y) = arr(new_no_of_row-1,y);
    end      
end
