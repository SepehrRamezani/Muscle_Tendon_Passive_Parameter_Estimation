close all
clear all
import org.opensim.modeling.*;
SubjectNumber='T003';
Project='P006';
Basepath=append(['C:\MyCloud\OneDriveUcf\Real\Simulation\Source'],'\',Project,'\',SubjectNumber);
load ([Basepath '\SimData.mat']);
OptTendonStrain=[];
Muscname=Data.SimMusclename;
MarkerSize=15;
OptPassiveFiber=[];


figure1=figure;
axes1 = axes('Parent',figure1);
hold(axes1,'on');
box(axes1,'on');
figure2=figure;
axes2 = axes('Parent',figure2);
hold(axes2,'on');
box(axes2,'on');
figure3=figure;
axes3 = axes('Parent',figure3);
hold(axes3,'on');
box(axes3,'on');
Lege=[];
newcolors = {'#F00','#F80','#FF0','#0B0','#00F','#50F','#A0F'};
for i=1:1:length(Data.Coordlable)
    %getting refrence data
    %Motion
RefState=TableProcessor(Data.(Data.Coordlable{i}).RefStatepath).process;
RefData.data=RefState.getMatrix().getAsMat();
RefData.colheaders=[];
for iLabel=0:RefState.getNumColumns()-1
    RefData.colheaders=[RefData.colheaders string(RefState.getColumnLabels.get(iLabel))];
end
reftime=[];
for iRow = 0 : RefState.getNumRows() - 1
    reftime(iRow+1,1) = RefState.getIndependentColumn.get(iRow);
end
% Torque
ControlDataTable=TableProcessor(Data.(Data.Coordlable{i}).RefControlpath).process;
ControlRefData.data=ControlDataTable.getMatrix().getAsMat();
ControlRefData.colheaders=[];
for iLabel=0:ControlDataTable.getNumColumns()-1
    ControlRefData.colheaders=[ControlRefData.colheaders string(ControlDataTable.getColumnLabels.get(iLabel))];
end
% getting optimization result 
u=0;
stiff_tendon_counter=0;
OptData=importdata(Data.(Data.Coordlable{i}).ParamSimulPath);


%% getting mucsles parameters 

    for m=1:length(Muscname)
        M_indx=find(contains(OptData.colheaders,Muscname(m))&~contains(OptData.colheaders,'force'));
        for y=1:length(M_indx)
            if contains(OptData.colheaders(M_indx(y)),'tendon_slack')
                OptTSL(i,m)=OptData.data(1,M_indx(y));
            elseif contains(OptData.colheaders(M_indx(y)),'passive_fiber_')
                stiff_tendon_counter=stiff_tendon_counter+1;
                OptPassiveFiber(i,stiff_tendon_counter)=OptData.data(1,M_indx(y));
            elseif contains(OptData.colheaders(M_indx(y)),'tendon_strain')
                u=u+1;
                OptTendonStrain(i,u)=OptData.data(1,M_indx(y));
            end
        end
        
    end
    %     Coordlable(i,1)=append("Hip",num2str(Data.Hipangle(i)));
    %% Calculating parameter error
    tendod_slack_ref=Data.(Data.Coordlable{i}).MuscleInfo.TSlack;
    tendon_slack_Error(i,:)=(OptTSL(i,:)-tendod_slack_ref)./tendod_slack_ref.*100;
     if OptPassiveFiber
        %     muscle_stiffness=1./(1+OptParam2(i,:));
        muscle_stiffness=2./(OptPassiveFiber(i,:));
        %     muscle_stiffness_ref=1./(1+muscle_passive_fiber_at_norm_ref);
        muscle_passive_fiber_at_norm_ref=Data.(Data.Coordlable{i}).MuscleInfo.Passive(contains(Muscname,Data.Rigidtendon));
        muscle_stiffness_ref=2./(muscle_passive_fiber_at_norm_ref);
        muscle_stiffness_error(i,:)=(muscle_stiffness-muscle_stiffness_ref)./muscle_stiffness_ref.*100;
        muscle_passive_fiber_at_norm_ref_Error(i,:)=100*(OptPassiveFiber(i,:)-muscle_passive_fiber_at_norm_ref)./muscle_passive_fiber_at_norm_ref;

     end
     if OptTendonStrain
         c1=0.2;
         c2=1;
         c3=0.2;
         tendon_stiffness = 1.1*log((1.0 + c3) / c1) ./ (1.0 + OptTendonStrain(i,:) - c2);
         %tendon_stiffness=1./(1+OptParam3(i,:));
         tendon_strain_at_norm_ref=Data.(Data.Coordlable{i}).MuscleInfo.Tstrain;
         %tendon_stiffness_ref=1./(1+tendon_strain_at_norm_ref);
         tendon_stiffness_ref=1.1*log((1.0 + c3) / c1) ./ (1.0 + tendon_strain_at_norm_ref - c2);
         TkErrortendon_stiffness_error(i,:)=(tendon_stiffness-tendon_stiffness_ref)./tendon_stiffness_ref.*100;
         epsilonerror(i,:)=(OptTendonStrain(i,:)-tendon_strain_at_norm_ref)./tendon_strain_at_norm_ref.*100;
     end
     
    
 %% Getting ref and FD states       
    Header=OptData.colheaders;
    Ddata=OptData.data;
    time=Ddata(:,1);
%     for corindx = 1:length(Data.ActiveCoordinates)
        % Angle
        Angle_idx=find(contains(Header,append(Data.ActiveCoordinates,'/value')));
        Activecoor=Ddata(:,Angle_idx)*180/3.14; 
        RefAngle_idx=(contains(RefData.colheaders,append(Data.ActiveCoordinates,'/value')));
        RefMotionActivecoor=RefData.data(:,RefAngle_idx)*180/3.14;
        % Toruqe
        act_idx=find(contains(Header,append(Data.ActiveCoordinates,'_act')));
        TorqueActivecoor=Ddata(:,act_idx);
        Ref_act_idx=(contains(ControlRefData.colheaders,append(Data.ActiveCoordinates,'_act')));
        RefTorqueActivecoor=ControlRefData.data(:,Ref_act_idx);
        
%     end

    vq1 = interp1(time,Activecoor,reftime);
    vq2 = (interp1(time,TorqueActivecoor,reftime))*Data.optForce  ;
    Error=RefMotionActivecoor-vq1;



color=newcolors{i};
plot(reftime,Error,'Color',color,'Parent',axes1);
plot(reftime,RefMotionActivecoor,'--',reftime,vq1,'Color',color,'Parent',axes2);
plot(reftime,RefTorqueActivecoor*Data.optForce,'--',reftime,vq2,'Color',color,'Parent',axes3)
Lege=[Lege ,append(Data.Coordlable{i},"-Exp"), append(Data.Coordlable{i},"-Sim")];
end
hold(axes1,'off');
hold(axes2,'off');
hold(axes3,'off');

ylabel(axes1,'Error(Deg)')
xlabel(axes1,'Time (s)')
title(axes1,'Knee angle')
legend(axes1,Data.Coordlable,'Location','southeast')
 
ylabel(axes2,'Angle (Deg)')
xlabel(axes2,'Time (s)')
title(axes2,'Knee angle')
legend(axes2,Lege,'Location','southeast')

ylabel(axes3,'Torque (N.m)')
xlabel(axes3,'Time (s)')
title(axes3,'Knee')
colororder(axes3,newcolors)
legend(axes3,Lege,'Location','southeast')
figure


% xlabel ('Knee Angle(Deg)')
% ylabel ('External Torque(N.m)')
% legend(Name,'Location','southeast')

% end
tiledlayout(2,2)

%% Tendon Slack Length
nexttile
X = categorical(Muscname);
plot(X,tendon_slack_Error,'.','MarkerSize',MarkerSize)
ylabel ('Tendon Slack length Estimation Error Compare to Rajagopal model(%)')
legend(Data.Coordlable,'Location','southeast')

%% Muscle Stiffness
if OptPassiveFiber
nexttile
X = categorical(Data.Rigidtendon);
plot(X,muscle_stiffness_error','.','MarkerSize',MarkerSize)
ylabel (["Muscle Stiffness Estimation Error Compare to Rajagopal model(%)"])
% legend(Data.Coordlable,'Location','southeast')
nexttile
X = categorical(Data.Rigidtendon);
plot(X,muscle_passive_fiber_at_norm_ref_Error','.','MarkerSize',MarkerSize)
ylabel ('Muscle passive force at one norm Error Compare to Rajagopal model(%)')

end
%% Tendon stiffness
if OptTendonStrain
nexttile
X2 = categorical(Data.ComplianacMusclename);
plot(X2,TkErrortendon_stiffness_error','.','MarkerSize',MarkerSize)
ylabel ('Tendon Stiffness Estimation Error(%)')
% legend(Data.Coordlable,'Location','northeast')
nexttile

X2 = categorical(Data.ComplianacMusclename);
plot(X2,epsilonerror','.','MarkerSize',MarkerSize)
ylabel ('Epsilon Estimation Error(%)')
legend(Data.Coordlable,'Location','northeast')
end



