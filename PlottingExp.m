close all
clear all
import org.opensim.modeling.*;
Project='P006';
txtBasepath=fullfile('C:\MyCloud\OneDriveUcf\Real\Simulation\Source',Project);
load ([txtBasepath '\SimData.mat']);
% Basepath=fullfile('C:\MyCloud\OneDriveUcf\Real\Simulation\Source',Project,SubjectNumber);
OptTendonStrain=[];
% Muscname=Data.SimMusclename;
MarkerSize=6;
OptPassiveFiber=[];
% SubjectNumber=["06","07","08","09","10","11","12","13","14","15"];
SubjectNumber=["10"];
SubjectNumber=append('T0',SubjectNumber);
% Joints=["Knee","Ankle"];
Joints=["Ankle","Knee"];
Joints=append("L",Joints);
Data.runver
% Terials2=["K90"];
Terials3=["L1","L2","L3"];

% figure1=figure;
% axes1 = axes('Parent',figure1);
% hold(axes1,'on');
% box(axes1,'on');
figure2=figure;
axes2 = axes('Parent',figure2);
hold(axes2,'on');
box(axes2,'on');
figure3=figure;
axes3 = axes('Parent',figure3);
hold(axes3,'on');
box(axes3,'on');
% for i=1:7
%    figures{i}=figure; 
% end
Lege=[];
newcolors = {'#F00','#F80','#000','#0B0','#00F','#50F','#A0F'};
fildnames=fieldnames(Data);
trialnames=string(fildnames((contains(fieldnames(Data),'_L1')|contains(fieldnames(Data),'_L2')|contains(fieldnames(Data),'_L3'))));
for S=1:length(SubjectNumber)

    psname=append(Project,'_',SubjectNumber(S));
    counter=0;
    OptPassiveFiber=[];
    OptTSL=[];
    OptTendonStrain=[];
    tendod_slack_rigid=[];
    tendod_slack_comp=[];
    tendon_slack_Error_comp=[];
    tendod_slack_ref=[];
    tendon_slack_Error_rigid=[];
    muscle_stiffness=[];
    muscle_stiffness_error=[];
    tendon_stiffness=[];
    tendon_stiffness_error=[];
    OptTSL=[];
    colorindx=0;
    for T1=1:length(Joints)

        if contains(Joints(T1),"Knee")
            Terials2=["H90","H55","H15"];
            kneelable=["H90","H55","H15"];
        else
            Terials2=["K90","K45","K0"];
            anklelable=["K90","K45","K0"];
            %             Terials2=["K90"];
        end
   
        counter=0;
        for T2=1:length(Terials2)
            
            
            combinedname=append(psname,'_',Joints(T1),'_',Terials2(T2));
            for T3=1:length(Terials3)
                counter=counter+1;
                trialname=append(psname,'_',Joints(T1),'_',Terials2(T2),'_',Terials3(T3));
                %getting refrence data
                %Motion
                
                RefState=TableProcessor(Data.(trialname).RefStatepath).process;
                RefData.data=RefState.getMatrix().getAsMat();
                RefData.colheaders=[];
                for iLabel=0:RefState.getNumColumns()-1
                    RefData.colheaders=[RefData.colheaders string(RefState.getColumnLabel(iLabel))];
                end
                reftime=[];
                for iRow = 0 : RefState.getNumRows() - 1
                    reftime(iRow+1,1) = RefState.getIndependentColumn.get(iRow);
                end
                % Torque
                ControlDataTable=TableProcessor(Data.(trialname).RefControlpath).process;
                ControlRefData.data=ControlDataTable.getMatrix().getAsMat();
                ControlRefData.colheaders=[];
                for iLabel=0:ControlDataTable.getNumColumns()-1
                    ControlRefData.colheaders=[ControlRefData.colheaders string(ControlDataTable.getColumnLabel(iLabel))];
                end
                % getting optimization result
                OptL_counter=0;
                TSlack_counter=0;
                compliance_tendon_counter=0;
                stiff_tendon_counter=0;
                OptTableData=TableProcessor(Data.(trialname).ParamSimulPath).process;
                OptData.data=OptTableData.getMatrix().getAsMat();
                Optreftime=[];
                for iRow = 0 : OptTableData.getNumRows() - 1
                    Optreftime(iRow+1,1) = OptTableData.getIndependentColumn.get(iRow);
                end
                OptData.colheaders=[];
                for iLabel=0:OptTableData.getNumColumns()-1
                    OptData.colheaders=[OptData.colheaders string(OptTableData.getColumnLabel(iLabel))];
                end
                if contains(string(OptTableData.getTableMetaDataAsString('success')),'true')
                    status=1;
                else
                    status=0;
                    fprintf('optimization %s is unsuccessful \n',trialname);
                end
                %% getting mucsles parameters
                c1=0.2;
                c2=1;
                c3=0.2;
                Muscname=Data.(combinedname).muscle4opt;
                MuscleInfo=Data.(combinedname).MuscleInfo;
                for m=1:length(Muscname)
                    M_indx=find(contains(OptData.colheaders,Muscname(m))&~contains(OptData.colheaders,'force'));
                    indxn=find(contains(MuscleInfo.Musclename,Muscname(m)));
                    
                    for y=1:length(M_indx)
                        if contains(OptData.colheaders(M_indx(y)),'tendon_slack')
                            TSlack_counter=TSlack_counter+1;
                            tendod_slack_ref(counter,m)= MuscleInfo.TSlack(indxn);
                            OptTSL(counter,m)=OptData.data(1,M_indx(y));
                            TSL_legend(counter,m)=MuscleInfo.Musclename(indxn);

                            tendon_slack_Error_rigid(counter,m)=(OptTSL(counter,m)-tendod_slack_ref(counter,m))./tendod_slack_ref(counter,m).*100;
                        elseif contains(OptData.colheaders(M_indx(y)),'passive_fiber')
                            stiff_tendon_counter=stiff_tendon_counter+1;
                            muscle_passive_fiber_at_norm_ref(counter,m)= MuscleInfo.Passive(indxn);
                            MusclepassiveOptmaxiso(counter,m)=MuscleInfo.MaxIso(indxn);
                            OptPassiveFiber(counter,stiff_tendon_counter)=OptData.data(1,M_indx(y));
                            
                            PassiveFiber_legend(1,m)=MuscleInfo.Musclename(indxn);

                            muscle_stiffness(counter,m)=2./(OptPassiveFiber(counter,m));
                            muscle_stiffness_ref(counter,m)=2./(muscle_passive_fiber_at_norm_ref(counter,m));
                            muscle_stiffness_error(counter,m)=(muscle_stiffness(counter,m)-muscle_stiffness_ref(counter,m))./muscle_stiffness_ref(counter,m).*100;
                            muscle_passive_fiber_at_norm_ref_Error(counter,m)=100*(OptPassiveFiber(counter,m)-muscle_passive_fiber_at_norm_ref(counter,m))./muscle_passive_fiber_at_norm_ref(counter,m);
                        elseif contains(OptData.colheaders(M_indx(y)),'tendon_strain')
                            compliance_tendon_counter=compliance_tendon_counter+1;
                            tendon_strain_at_norm_ref(counter,m)=MuscleInfo.Tstrain(indxn);
                            OptTendonStrain(counter,compliance_tendon_counter)=OptData.data(1,M_indx(y));
                            TendonStrain_legend(counter,m)=MuscleInfo.Musclename(indxn);
                            Tenodoptmaxiso(counter,m)=MuscleInfo.MaxIso(indxn);
                            TenodoptOptFiber(counter,m)=MuscleInfo.OptFiberL(indxn);
                            tendon_stiffness(counter,m) = 1.1*log((1.0 + c3) / c1) ./ (1.0 + OptTendonStrain(counter,m) - c2);
                            tendon_stiffness_ref(counter,m)=1.1*log((1.0 + c3) / c1) ./ (1.0 + tendon_strain_at_norm_ref(counter,m) - c2);
                            tendon_stiffness_error(counter,m)=(tendon_stiffness(counter,m)-tendon_stiffness_ref(counter,m))./tendon_stiffness_ref(counter,m).*100;
                            epsilonerror(counter,m)=(OptTendonStrain(counter,m)-tendon_strain_at_norm_ref(counter,m))./tendon_strain_at_norm_ref(counter,m).*100;

                        elseif contains(OptData.colheaders(M_indx(y)),'OptL')
                            OptL_counter=OptL_counter+1;
                            Opt_fiber_length_ref(counter,m)=MuscleInfo.OptFiberL(indxn);
                            OptFiberLength(counter,OptL_counter)=OptData.data(1,M_indx(y));
                            OptFiberLength_legend(counter,m)=MuscleInfo.Musclename(indxn);
                            Opt_fiber_length_Error(counter,m)=(OptFiberLength(counter,m)-Opt_fiber_length_ref(counter,m))./Opt_fiber_length_ref(counter,m).*100;

                        end
                    end

                end
             


                %% Getting ref and FD states
                Header=OptData.colheaders;
                Ddata=OptData.data;
                time=Optreftime;
                %     for corindx = 1:length(Data.ActiveCoordinates)
                % Angle
                ActiveCoordinates=Data.(combinedname).ActiveCoordinates;
                Angle_idx=find(contains(Header,append(ActiveCoordinates,'/value')));
                Activecoor=Ddata(:,Angle_idx)*180/3.14;
                RefAngle_idx=(contains(RefData.colheaders,'Biodex'));
                RefMotionActivecoor=RefData.data(:,RefAngle_idx)*180/3.14;
                % Toruqe
                act_idx=find(contains(Header,append(ActiveCoordinates,'_act')));
                TorqueActivecoor=Ddata(:,act_idx);
                Ref_act_idx=(contains(ControlRefData.colheaders,append(ActiveCoordinates,'_act')));
                RefTorqueActivecoor=ControlRefData.data(:,Ref_act_idx);

                %     end

                vq1 = interp1(time,Activecoor,reftime);
                vq2 = (interp1(time,TorqueActivecoor,reftime))*Data.optForce  ;
                Error=RefMotionActivecoor-vq1;
                colorindx=colorindx+1;
                if colorindx > 7 colorindx=1; end
                color=newcolors{colorindx};
%                 plot(reftime,Error,'Color',color,'Parent',axes1);
                plot(reftime,RefMotionActivecoor,'--',reftime,vq1,'Color',color,'Parent',axes2);
                legend([append(trialname,"-Exp"), append(trialname,"-Sim")]);
                plot(reftime,RefTorqueActivecoor*Data.optForce,'--',reftime,vq2,'Color',color,'Parent',axes3)
                legend([append(trialname,"-Exp"), append(trialname,"-Sim")]);
            end
%             Tendod_slacks.(Terials2(T2))=table2cell(table(Muscname',tendod_slack'));
%             muscle_stiffnesss.(Terials2(T2))=table2cell(table(Muscname',muscle_stiffness'));
%             tendod_slack=
            
        end
        
        
        MutilOptPassiveFiber.(Joints(T1)).OptPassiveFiber=OptPassiveFiber;
        MutilOptPassiveFiber.(Joints(T1)).muscle_stiffness=muscle_stiffness;
        MutilOptPassiveFiber.(Joints(T1)).muscle_stiffness_ref=muscle_stiffness_ref;
        MutilOptPassiveFiber.(Joints(T1)).muscle_stiffness_error=muscle_stiffness_error;
        MutilOptPassiveFiber.(Joints(T1)).MaxIso=MusclepassiveOptmaxiso;
        MutilOptPassiveFiber.(Joints(T1)).Legend=PassiveFiber_legend;
        
    end
    
    %% Tendon Slack Length
%     tendod_slack=[tendod_slack_rigid,tendod_slack_comp];

    h(1)=figure;
    ttt=append(SubjectNumber(S),'-Tendon Slack length different trials');
    [rl,cl]=size(TSL_legend);
    t=tiledlayout(ceil(cl/2),2);
    multiplotting(t,TSL_legend(1,:),anklelable,ttt,OptTSL,newcolors,MarkerSize,1);
   %% Muscle Stiffness
    h(2)=figure;
    cl=10;
    t2=tiledlayout(ceil(cl/2),2);
    ttt2=append(SubjectNumber(S),'Muscle Stiffness different trials');
    if ~contains(Joints(1),'Ankle')
        Joints(1:end)=Joints(end:-1:1);
    end
    musclestiffnesscoef=Tenodoptmaxiso./TenodoptOptFiber;
    PData=MutilOptPassiveFiber.(Joints(1)).muscle_stiffness.*musclestiffnesscoef;
    Ylable=MutilOptPassiveFiber.(Joints(1)).Legend;
    multiplotting(t2,Ylable,anklelable,ttt2,PData,newcolors,MarkerSize,1);
    
    musclestiffnesscoef1=MutilOptPassiveFiber.(Joints(2)).MaxIso./MutilOptPassiveFiber.(Joints(2)).OptPassiveFiber;
    PData=MutilOptPassiveFiber.(Joints(2)).muscle_stiffness.*musclestiffnesscoef1;
    Ylable=MutilOptPassiveFiber.(Joints(2)).Legend;
    multiplotting(t2,Ylable,kneelable,ttt2,PData,newcolors,MarkerSize,1);
    %% tendon Stiffness
    tendocoef=Tenodoptmaxiso./OptTSL;
    h(2)=figure;
    ttt2=append(SubjectNumber(S),'Tendon Stiffness different trials');
    c1=4;
    t2=tiledlayout(ceil(cl/2),2);
    PData=tendon_stiffness.*tendocoef;
    Ylable=TendonStrain_legend(1,:);
    multiplotting(t2,Ylable,anklelable,ttt2,PData,newcolors,MarkerSize,1);
    %%
    h(4)=figure;
    MarkerSize=10;
    t3=tiledlayout(2,2);
    nexttile
    titet=SubjectNumber(S);
    Ylable="Tendon Slack length(m)";
    multiplotting(t3,Ylable,Muscname_rigid_tendon,titet,tendod_slack_rigid,newcolors,MarkerSize,0);
    legend(kneelable)
    nexttile   
    multiplotting(t3,Ylable,Muscname_comp_tendon,titet,tendod_slack_comp,newcolors,MarkerSize,0);
    legend(anklelable);
    nexttile
    Ylable="Muscle Stiffness(N.m)";
    multiplotting(t3,Ylable,Muscname_rigid_tendon,titet,muscle_stiffness,newcolors,MarkerSize,0);
    legend(kneelable);
    nexttile
    Ylable="Tendon Stiffness(N.m)";
    multiplotting(t3,Ylable,Muscname_comp_tendon,titet,tendon_stiffness,newcolors,MarkerSize,0);
    legend(anklelable);
    resdir=fullfile(txtBasepath,SubjectNumber(S),'Result',append(SubjectNumber(S),'-Muscle-Tendo-',Data.runver,'.fig'));
    savefig(h,resdir)
    fprintf('%s is done \n',SubjectNumber(S));
end
%     legend(Terials3);


% hold(axes1,'off');
% hold(axes2,'off');
% hold(axes3,'off');
%
% ylabel(axes1,'Error(Deg)')
% xlabel(axes1,'Time (s)')
% title(axes1,'Knee angle')
% legend(axes1,Data.Coordlable,'Location','southeast')
%
% ylabel(axes2,'Angle (Deg)')
% xlabel(axes2,'Time (s)')
% title(axes2,'Knee angle')
% legend(axes2,Lege,'Location','southeast')
    %
    % ylabel(axes3,'Torque (N.m)')
    % xlabel(axes3,'Time (s)')
    % title(axes3,'Knee')
    % colororder(axes3,newcolors)
    % legend(axes3,Lege,'Location','southeast')
    % figure
    % xlabel ('Knee Angle(Deg)')
    % ylabel ('External Torque(N.m)')
    % legend(Name,'Location','southeast')


    
        
 


