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
SubjectNumber=["06","07","08","09","10","11","12","13","14","15","R"];
% SubjectNumber=["06","R"];
SubjectNumber=append('T0',SubjectNumber);
Joints=["Ankle","Knee"];
% Joints=["Ankle"];
Joints=append("L",Joints);
% Data.runver="E_Sim";
torqueflage=0;
% Terials2=["K90"];
Terials3=["L1","L2","L3"];
% Terials3=["L1"];
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
runver='E16';
refflag=0;
[objectvaluetable1,TSLtable1,MStable1,TStable1,epsilontable1,Musclenametable1,subjectnametable,TSLNorm1]=deal([]);
for S=1:length(SubjectNumber)
    [objectvaluetable,TSLtable,MStable,TStable,epsilontable,Musclenametable,TSLNorm]=deal([]);
     if contains(SubjectNumber(S),"T0R")
         refflag=1;
         SubjectNumber(S)="T006";
     end

        psname=append(Project,'_',SubjectNumber(S));
            Basepath=append('C:\MyCloud\OneDriveUcf\Real\Simulation\Source','\',Project,'\',SubjectNumber(S));

    counter=0;
    OptPassiveFiber=[];
    OptTSL=[];
    OptTendonStrain=[];
    [tendod_slack_rigid, tendod_slack_comp,tendon_slack_Error_comp,tendod_slack_ref,tendon_slack_Error_rigid,tendod_slack,tendod_slack_norm]=deal([]);
    [muscle_stiffness,muscle_stiffness_error,muscle_stiffness_ref,tendon_stiffness,tendon_stiffness_error,passive_fiber_at_norm]=deal([]);
    OptTSL=[];
    objectvalue=0;
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
        Muscle_legend="";
        counter=0;
        for T2=1:length(Terials2)


            combinedname=append(psname,'_',Joints(T1),'_',Terials2(T2));
            Data.(combinedname).Modelpath=append(Basepath,'\Model\',combinedname,'_',runver,'.osim');
            combinedname=append(psname,'_',Joints(T1),'_',Terials2(T2));
            Refmmodel = Model(char(Data.(combinedname).Modelpath));
            
            Anklecoordref = Refmmodel.getCoordinateSet().get('ankle_angle_l');
            Kneecoordref = Refmmodel.getCoordinateSet().get('knee_angle_l');
            %                 Currentcoordref=Refmmodel.updCoordinateSet().get(Data.(combinedname).ActiveCoordinates);
            state=Refmmodel.initSystem();
            if contains(Joints(T1),"Knee")
                hipjointref=Refmmodel.getJointSet().get('hip_l');
                pelvisparentframeref=hipjointref.upd_frames(0);
                pelvisparentframeref.set_orientation(Vec3(0,0,0));
                lenghtscale=Refmmodel.getBodySet().get('femur_l').get_attached_geometry(0).get_scale_factors().get(0)*0.40;
            else
                lenghtscale=Refmmodel.getBodySet().get('tibia_l').get_attached_geometry(0).get_scale_factors().get(0)*0.40;
            end
            Anklecoordref.setDefaultValue(0);
            Kneecoordref.setDefaultValue(0);
            %                 angle(k,i+1)=Currentcoord.getValue(state);
            Refmmodel.realizePosition(state);
            Refmmodel.equilibrateMuscles(state);

            objectvaluelocal=0;
            for T3=1:length(Terials3)
                counter=counter+1;
                trialname=append(psname,'_',Joints(T1),'_',Terials2(T2),'_',Terials3(T3));
                %getting refrence data
                %Motion


                Data.(trialname).RefStatepath=append(Basepath,'\Data\',trialname,'_Motion.mot');
                Data.(trialname).RefControlpath=append(Basepath,'\Data\',trialname,'_Torque.sto');
                paramdir=fullfile(Basepath,'Parameterestimation');
                Data.(trialname).TorqeSimulPath=append(paramdir,'\Torque_Opt_',trialname,'_',runver,'.sto');
                Data.(trialname).ParamSimulPath=append(paramdir,'\Parameter_Opt_',trialname,'_',runver,'.sto');
                Data.(trialname).ModelPath=append(Basepath,'\Model\',trialname,'_',runver,'.osim');

                if torqueflage
                    RefState=TableProcessor(Data.(trialname).TorqeSimulPath).process;
                    ControlDataTable=TableProcessor(Data.(trialname).TorqeSimulPath).process;
                else
                    RefState=TableProcessor(Data.(trialname).RefStatepath).process;
                    ControlDataTable=TableProcessor(Data.(trialname).RefControlpath).process;
                end

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
                % getTableMetaDataKeys
                objectvalue(counter)=double(string(OptTableData.getTableMetaDataAsString('objective')));
                objectvaluelocal(T3)=objectvalue(counter);
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
                    
                if refflag
                    OPtimizedmmodel=Model(append(Basepath,'\..\P006_refmodel_Rajagopal_Degroote.osim'));
                else
                    OPtimizedmmodel = Model(insertBefore(char(Data.(trialname).ModelPath),".osim","_optimized"));
                end
                state2=OPtimizedmmodel.initSystem();
                modeljointSet = OPtimizedmmodel.getCoordinateSet();
                Kneecoord = modeljointSet.get('ankle_angle_l');
                Anklecoord = modeljointSet.get('knee_angle_l');
                if contains(Joints(T1),"Knee")
                    hipjoint=OPtimizedmmodel.getJointSet.get('hip_l');
                    pelvisparentframe=hipjoint.upd_frames(0);
                    pelvisparentframe.set_orientation(Vec3(0,0,0));
                end

                Anklecoord.setDefaultValue(0);
                Kneecoord.setDefaultValue(0);
                OPtimizedmmodel.realizePosition(state2);
                OPtimizedmmodel.equilibrateMuscles(state2);

                for m=1:length(Muscname)
                    M_indx=find(contains(OptData.colheaders,Muscname(m))&~contains(OptData.colheaders,'force'));

                    %%%%%%%%%%%%%%%
                    RefCurrentMuscle=Refmmodel.getMuscles().get(Muscname(m));
                    Refdegmus=DeGrooteFregly2016Muscle.safeDownCast(RefCurrentMuscle);
                    Musclestiff_ref(counter,m)=Refdegmus.getMuscleStiffness(state);
                    Tendonstiff_ref(counter,m)=Refdegmus.getTendonStiffness(state);
                    tendod_slack_ref(counter,m)=Refdegmus.get_tendon_slack_length();
                    Opt_fiber_length_ref(counter,m)=Refdegmus.get_optimal_fiber_length();
                    passive_fiber_at_norm_ref(counter,m)=Refdegmus.get_passive_fiber_strain_at_one_norm_force();
                    tendon_strain_at_norm_ref(counter,m)= Refdegmus.get_tendon_strain_at_one_norm_force();

                    CurrentMuscle=OPtimizedmmodel.getMuscles().get(Muscname(m));
                    degmus=DeGrooteFregly2016Muscle.safeDownCast(CurrentMuscle);
                    muscle_stiffness(counter,m)=degmus.getMuscleStiffness(state2);
                    tendon_stiffness(counter,m)=degmus.getTendonStiffness(state2);
                    
                    tendod_slack(counter,m)=(degmus.get_tendon_slack_length());
                    tendod_slack_norm(counter,m)=tendod_slack(counter,m)./lenghtscale;
                    Opt_fiber_length(counter,m)=degmus.get_optimal_fiber_length();
                    passive_fiber_at_norm(counter,m)=degmus.get_passive_fiber_strain_at_one_norm_force();
                    tendon_strain_at_norm(counter,m)= degmus.get_tendon_strain_at_one_norm_force();

                    tendon_slack_Error_rigid_error(counter,m)=((tendod_slack(counter,m)./tendod_slack_ref(counter,m))-1)*100;
                    muscle_stiffness_error(counter,m)=(muscle_stiffness(counter,m)./Musclestiff_ref(counter,m)-1)*100;
                    tendon_stiffness_error(counter,m)=(tendon_stiffness(counter,m)./Tendonstiff_ref(counter,m)-1)*100;
                    epsilont_error(counter,m)=(tendon_strain_at_norm(counter,m)./tendon_strain_at_norm_ref(counter,m)-1)*100;
                    %%%%%%%%%%%%%%%%%%%%
                    %                     indxn=find(contains(MuscleInfo.Musclename,Muscname(m)));
                    Muscle_legend(counter,m)=Muscname(m);
                    %                     MusclepassiveOptmaxiso(counter,m)=MuscleInfo.MaxIso(indxn);
                    %                     Tenodoptmaxiso(counter,m)=MuscleInfo.MaxIso(indxn);
                    %                     tendod_slack_ref(counter,m)= MuscleInfo.TSlack(indxn);
                    %                     tendon_strain_at_norm_ref(counter,m)=MuscleInfo.Tstrain(indxn);
                    %                     muscle_passive_fiber_at_norm_ref(counter,m)= MuscleInfo.Passive(indxn);
                    %                     Opt_fiber_length_ref(counter,m)=MuscleInfo.OptFiberL(indxn);


                    %                     for y=1:length(M_indx)
                    %                         if contains(OptData.colheaders(M_indx(y)),'tendon_slack')
                    %                             TSlack_counter=TSlack_counter+1;
                    %                             OptTSL(counter,m)=OptData.data(1,M_indx(y));
                    %                             tendon_slack_Error_rigid(counter,m)=(OptTSL(counter,m)-tendod_slack_ref(counter,m))./tendod_slack_ref(counter,m).*100;
                    %                         elseif contains(OptData.colheaders(M_indx(y)),'passive_fiber')
                    %                             stiff_tendon_counter=stiff_tendon_counter+1;
                    %                             OptPassiveFiber(counter,stiff_tendon_counter)=OptData.data(1,M_indx(y));
                    % %                             PassiveFiber_legend(1,m)=MuscleInfo.Musclename(indxn);
                    %                             muscle_stiffness(counter,m)=2./(OptPassiveFiber(counter,m));
                    %                             muscle_stiffness_ref(counter,m)=2./(muscle_passive_fiber_at_norm_ref(counter,m));
                    %                             muscle_stiffness_error(counter,m)=(muscle_stiffness(counter,m)-muscle_stiffness_ref(counter,m))./muscle_stiffness_ref(counter,m).*100;
                    %                             muscle_passive_fiber_at_norm_ref_Error(counter,m)=100*(OptPassiveFiber(counter,m)-muscle_passive_fiber_at_norm_ref(counter,m))./muscle_passive_fiber_at_norm_ref(counter,m);
                    %                         elseif contains(OptData.colheaders(M_indx(y)),'tendon_strain')
                    %                             compliance_tendon_counter=compliance_tendon_counter+1;
                    %                             OptTendonStrain(counter,compliance_tendon_counter)=OptData.data(1,M_indx(y));
                    %                             tendon_strain_at_norm_ref_Err(counter,compliance_tendon_counter)=((OptTendonStrain(counter,compliance_tendon_counter)/tendon_strain_at_norm_ref(counter,m))-1)*100;
                    %                             tendon_stiffness(counter,m) = 1.1*log((1.0 + c3) / c1) ./ (1.0 + OptTendonStrain(counter,m) - c2);
                    %                             tendon_stiffness_ref(counter,m)=1.1*log((1.0 + c3) / c1) ./ (1.0 + tendon_strain_at_norm_ref(counter,m) - c2);
                    %                             tendon_stiffness_error(counter,m)=(tendon_stiffness(counter,m)-tendon_stiffness_ref(counter,m))./tendon_stiffness_ref(counter,m).*100;
                    %                             epsilonerror(counter,m)=(OptTendonStrain(counter,m)-tendon_strain_at_norm_ref(counter,m))./tendon_strain_at_norm_ref(counter,m).*100;
                    %
                    %                         elseif contains(OptData.colheaders(M_indx(y)),'OptL')
                    %                             OptL_counter=OptL_counter+1;
                    %                             OptFiberLength(counter,OptL_counter)=OptData.data(1,M_indx(y));
                    %                             OptFiberLength_legend(counter,m)=MuscleInfo.Musclename(indxn);
                    %                             Opt_fiber_length_Error(counter,m)=(OptFiberLength(counter,m)-Opt_fiber_length_ref(counter,m))./Opt_fiber_length_ref(counter,m).*100;
                    %
                    %                         end
                    %                      end



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
                if torqueflage
                    RefAngle_idx=(contains(RefData.colheaders,append(ActiveCoordinates,'/value')));
                else
                    RefAngle_idx=(contains(RefData.colheaders,'Biodex'));
                end

                RefMotionActivecoor=RefData.data(:,RefAngle_idx)*180/3.14;
                % Toruqe
                act_idx=find(contains(Header,append(ActiveCoordinates,'_act')));
                TorqueActivecoor=Ddata(:,act_idx);
                Ref_act_idx=(contains(ControlRefData.colheaders,append(ActiveCoordinates,'_act')));
                RefTorqueActivecoor=ControlRefData.data(:,Ref_act_idx);

                %     end

                vq1 = interp1(time,Activecoor,reftime);
                vq2 = (interp1(time,TorqueActivecoor,reftime))*Data.optForce;
                ErrorMotion=RefMotionActivecoor-vq1;
                ErrorTorque=RefTorqueActivecoor*Data.optForce-vq2;
                colorindx=colorindx+1;
                if colorindx > 7 colorindx=1; end
                color=newcolors{colorindx};
                %                 plot(reftime,ErrorMotion,'Parent',axes3);
                %                 xlabel("Time(s)")
                %                 ylabel("Motion Error (deg)")
                %                 plot(reftime,ErrorTorque,'Parent',axes2);
%                 xlabel("Time(s)")
%                 ylabel("Torque Error (N.m)")
                %                 plot(reftime,RefMotionActivecoor,'--',reftime,vq1,'Color',color,'Parent',axes2);
                %                 legend([append(trialname,"-Exp"), append(trialname,"-Sim")]);
                %                 plot(reftime,RefTorqueActivecoor*Data.optForce,'--',reftime,vq2,'Color',color,'Parent',axes3)
                %                 legend([append(trialname,"-Exp"), append(trialname,"-Sim")]);

            end
            minObjVlaue(T2)=min(objectvaluelocal);
            
            %             Tendod_slacks.(Terials2(T2))=table2cell(table(Muscname',tendod_slack'));
            %             muscle_stiffnesss.(Terials2(T2))=table2cell(table(Muscname',muscle_stiffness'));
            %             tendod_slack=

        end

        %         figure
        minminObjValue=min(minObjVlaue);
        minminindx=find(objectvalue==minminObjValue);
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).minObjVlaue=minObjVlaue;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).ObjVlaue=objectvalue;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).TSL=tendod_slack;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).TSLNorm=tendod_slack_norm;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).TSLError=tendon_slack_Error_rigid_error;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).OptPassiveFiber=OptPassiveFiber;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).muscle_stiffness=muscle_stiffness;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).tendon_stiffness_error=tendon_stiffness_error;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).tendon_stiffness=tendon_stiffness;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).muscle_stiffness_error=muscle_stiffness_error;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).epsilont_error=epsilont_error;
        %         MutilOptPassiveFiber.(Joints(T1)).MaxIso=MusclepassiveOptmaxiso;
        ResultOpt.(SubjectNumber(S)).(Joints(T1)).Legend=Muscle_legend;
        %         legend(Terials2)
        Musclenametable=[Musclenametable Muscle_legend(minminindx(1),:)];
        if contains(Joints(T1),"Knee")
        else
            TStable=[TStable tendon_stiffness(minminindx(1),:)];

        end
        epsilontable=[epsilontable passive_fiber_at_norm(minminindx(1),:)];

        MStable=[MStable muscle_stiffness(minminindx(1),:)];

        TSLtable=[TSLtable tendod_slack(minminindx(1),:)];
        TSLNorm=[TSLNorm tendod_slack_norm(minminindx(1),:)];
        objectvaluetable=[objectvaluetable minminObjValue];

    end

    epsilontable1=[epsilontable1;epsilontable];
    MStable1=[MStable1;MStable];
    TStable1=[TStable1;TStable];
    TSLtable1=[TSLtable1;TSLtable];
    TSLNorm1=[TSLNorm1;TSLNorm];
    objectvaluetable1=[objectvaluetable1; objectvaluetable];
    Musclenametable1=[Musclenametable1; Musclenametable];
    subjectnametable=[subjectnametable; SubjectNumber(S)];


    %% Tendon Slack Length
    %     tendod_slack=[tendod_slack_rigid,tendod_slack_comp];
    if 0
        if ~contains(Joints(1),'Ankle')
            Joints(1:end)=Joints(end:-1:1);
        end
        if ~torqueflage
            h(1)=figure;
            ttt=[""];
            [rl,cl]=size(Muscle_legend);
            t=tiledlayout(ceil(cl/2),2);
            multiplotting(t,ResultOpt.("LKnee").Legend(1,:),kneelable,ttt,ResultOpt.("LKnee").TSL,newcolors,MarkerSize,1);
            %% Muscle Stiffness
            h(2)=figure;
            cl=10;
            t2=tiledlayout(ceil(cl/2),2);
            ttt2=append(SubjectNumber(S),'Muscle Stiffness different trials');

            %     PData=MutilOptPassiveFiber.("LAnkle").muscle_stiffness;
            %     Ylable=MutilOptPassiveFiber.("LAnkle").Legend(1,:);
            %     multiplotting(t2,Ylable,anklelable,ttt2,PData,newcolors,MarkerSize,1);

            PData=ResultOpt.("LKnee").muscle_stiffness;
            Ylable=ResultOpt.("LKnee").Legend(1,:);
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
        end
        h(4)=figure;
        MarkerSize=10;
        t3=tiledlayout(1,2);
        nexttile
        titet=[""];
        Ylable="Tendon Slack length Error(%)";

        multiplotting(t3,Ylable,ResultOpt.(Joints(2)).Legend(1,:),titet,ResultOpt.(Joints(2)).TSLError,newcolors,MarkerSize,0);
        legend(kneelable)
        nexttile
        multiplotting(t3,Ylable,ResultOpt.(Joints(1)).Legend(1,:),titet,ResultOpt.(Joints(1)).muscle_stiffness_error,newcolors,MarkerSize,0);
        legend(anklelable);



        h(5)=figure;
        t3=tiledlayout(1,2);
        Ylable="Muscle Stiffness Error(%)";
        nexttile
        multiplotting(t3,Ylable,ResultOpt.(Joints(2)).Legend(1,:),titet,ResultOpt.(Joints(2)).muscle_stiffness_error,newcolors,MarkerSize,0);
        legend(kneelable);
        nexttile
        multiplotting(t3,Ylable,ResultOpt.(Joints(1)).Legend(1,:),titet,ResultOpt.(Joints(1)).TSLError,newcolors,MarkerSize,0);
        legend(anklelable);


        h(6)=figure;
        t3=tiledlayout(1,2);
        Ylable="Tendon Stiffness Error(%)";
        multiplotting(t3,Ylable,ResultOpt.(Joints(1)).Legend(1,:),titet,ResultOpt.(Joints(1)).tendon_stiffness_error,newcolors,MarkerSize,0);
        legend(anklelable);
        resdir=fullfile(txtBasepath,SubjectNumber(S),'Result',append(SubjectNumber(S),'-Muscle-Tendo-',Data.runver,'.fig'));
        savefig(h,resdir)
        fprintf('%s is done \n',SubjectNumber(S));
    end

end

%% plotting tendon slack length
X1=categorical(reshape(erase(Musclenametable1,"_l"),[110,1]),'Ordinal',false);
Y1=reshape(TSLNorm1,[110,1]);

% X3 = categorical(Musclenametable);
boxchart(X1,Y1,'MarkerStyle','.','LineWidth',1.5);
hold on
[B,I]=sort(Musclenametable1(1,:));
p1=plot(TSLNorm1(:,I)','o','LineWidth',0.25);
% scatter1 = scatter(X1,Y1,'MarkerEdgeAlpha',0.2,'ColorVariable','Diastolic');
hold off
legend(p1,replace(SubjectNumber,'T0','P'))
figure
tendon=sum(TStable1,2);
% X2=categorical(reshape(Musclenametable1(:,1:3),[30,1]));
% Y2=reshape(TStable1,[30,1]);
boxchart(tendon,'MarkerStyle','.','LineWidth',1.5);
hold on
p2=plot(1,tendon,'o');
legend(p2,replace(SubjectNumber,'T0','P'))
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







