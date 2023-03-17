clear all
import org.opensim.modeling.*;
% myLog = JavaLogSink();
% Logger.addSink(myLog);

Project='P006';
txtBasepath=fullfile('C:\MyCloud\OneDriveUcf\Real\Simulation\Source',Project);
load([txtBasepath '\SimData.mat'])
runver="E_Sim";
Data.runver=runver;
% SubjectNumber=["06","07","08","09","10","11","12","13","14","15"];
SubjectNumber=["10"];


SubjectNumber=append("T0",SubjectNumber);
Data.whichleg="l";
Data.joints=["ground_pelvis","hip","walker_knee","patellofemoral","ankle","mtp","subtalar"];
Data.joints(2:end)=addingleg(Data.joints(2:end),Data.whichleg);
Data.Weldjoints=["ground_pelvis","hip","mtp","subtalar"];
Data.Weldjoints(2:end)=addingleg(Data.Weldjoints(2:end),Data.whichleg);
Data.bodies=["pelvis","femur","tibia","patella","talus","calcn","toes"];
Data.bodies(2:end)=addingleg(Data.bodies(2:end),Data.whichleg);
Data.maxpanlt=-deg2rad(15);
Data.RajRigidMus=["bfsh","sart","grac"];
Data.RajRigidMus=addingleg(Data.RajRigidMus,Data.whichleg);
Data.DeGrooteflage=1;
% Hipangle=0;%deg
% Basepath=[cd];

% Data.RefStatepath=append(Basepath,'\TorqueSimulation\referenceCoordinates.sto');

Data.TorqueSolverinterval=40;
Data.ParamSolverinterval=50;
% Data.Etime=20;
Data.Stime=0;
Data.PassiveFiberBound=[0.5,1];
Data.TendonStrainBound=[0.03,0.09];


Joints=["Knee","Ankle"];
% Joints=["Ankle"];
% Terials3=["L1","L2","L3"];
 Terials3=["L1"];

diarydir=append(txtBasepath,"\log.txt");
% diary(diarydir)
%% running just Parameter optimization
Data.justparameterflag=1;
qe=1;
for S=1:length(SubjectNumber)
    Basepath=append('C:\MyCloud\OneDriveUcf\Real\Simulation\Source','\',Project,'\',SubjectNumber(S));
    
    Pardata=importdata(append(Basepath,"\Data\Parameters.csv"));
    psname=append(Project,'_',SubjectNumber(S));
    Data.optForce=Pardata.data(6);
    Data.(psname).RefModelpath=append(Basepath,'\Model\',psname,'_Rajagopal_Scaled.osim');
    
    for T1=1:length(Joints)
        if contains(Joints(T1),"Knee")
            Terials2=["H90","H55","H15"];
%             Terials2=["H90"];
            Terials2unmtxt=erase(Terials2,"H");
            Terials2unm=str2double(Terials2unmtxt);
            Terials2unm(find(Terials2unm==15))=10;
            Terials2unm=deg2rad(Terials2unm);
%             Terials2unm=deg2rad(10);
            Data.ActiveCoordinates=["knee_angle"];
            Data.Rigidtendon=["bflh","bfsh","recfem","semimem","semiten","vasint","vaslat","vasmed","sart","grac"];
            Data.ComplianceTendon=[];
            Data.muscle4opt=["bflh","recfem","semimem","semiten","vasint","vaslat","vasmed"];
        else
            Terials2=["K90","K45","K0"];
%             Terials2=["K0"];
            Terials2unmtxt=erase(Terials2,"K");
            Terials2unm=deg2rad(str2double(Terials2unmtxt));
            Data.ActiveCoordinates=["ankle_angle"];
            Data.Rigidtendon=["tibpost","perlong","perbrev","fdl","fhl"];
            Data.ComplianceTendon=["gaslat","gasmed","soleus"];
            Data.muscle4opt=["gaslat","gasmed","soleus"];
            Data.TendonStiffness=[108 390;108 390;108 390]*1000/3; %N/m
        end
        Data.ActiveCoordinates=addingleg(Data.ActiveCoordinates,Data.whichleg);
        Data.Rigidtendon=addingleg(Data.Rigidtendon,Data.whichleg);
        Data.muscle4opt=addingleg(Data.muscle4opt,Data.whichleg);
        Data.ComplianceTendon=addingleg(Data.ComplianceTendon,Data.whichleg);
        % Make sure if you have common name in the Rigidtendon and Compliance
        % Muscle put their name at the end of rigid tendon.
        if ~isempty(Data.ComplianceTendon)
            Data.SimMusclename=[Data.Rigidtendon(~contains(Data.Rigidtendon,Data.ComplianceTendon)),Data.ComplianceTendon];
        else
            Data.SimMusclename=Data.Rigidtendon;
        end
        for T2=1:length(Terials2)
            
            combinedname=append(psname,'_',upper(Data.whichleg),Joints(T1),'_',Terials2(T2));
            Data.(combinedname).Modelpath=append(Basepath,'\Model\',combinedname,'_',runver,'.osim');
            if contains(Joints(T1),"Knee")
                Data.(combinedname).Hipangle=Terials2unm(T2);
                Data.(combinedname).Kneeangle=0;
                Data.(combinedname).Ankleangle=0;
            else
                Data.(combinedname).Hipangle=Terials2unm(T2);
                Data.(combinedname).Kneeangle= Terials2unm(T2);
                Data.(combinedname).Ankleangle=deg2rad(30);
            end
             Refmmodel = Model(Data.(psname).RefModelpath);
             [osimmodel,Data.(combinedname).MuscleInfo]=Modelcreator(combinedname,Data,Refmmodel);
%               Passiveforcegenerate(combinedname,Data,Refmmodel);
             Data.(combinedname).SimMusclename=Data.SimMusclename;
             Data.(combinedname).ActiveCoordinates=Data.ActiveCoordinates;
             Data.(combinedname).ComplianceTendon=Data.ComplianceTendon;
             Data.(combinedname).muscle4opt=Data.muscle4opt;
            for T3=1:length(Terials3)
                filename=append(psname,'_',upper(Data.whichleg),Joints(T1),'_',Terials2(T2),'_',Terials3(T3));
                Data.(filename).RefStatepath=append(Basepath,'\Data\',filename,'_Motion.mot');
                Data.(filename).RefControlpath=append(Basepath,'\Data\',filename,'_Torque.sto');
                paramdir=fullfile(Basepath,'Parameterestimation');
                [r,e]=mkdir(paramdir);
                Data.(filename).TorqeSimulPath=append(paramdir,'\Torque_Opt_',filename,'_',runver,'.sto');
                Data.(filename).ParamSimulPath=append(paramdir,'\Parameter_Opt_',filename,'_',runver,'.sto');
                Data.(filename).ModelPath=append(Basepath,'\Model\',filename,'_',runver,'.osim');
                StateDataTable=TableProcessor(Data.(filename).RefStatepath).process;
                HipAngle=StateDataTable.getDependentColumnAtIndex(0).getAsMat();
                KneeAngle=StateDataTable.getDependentColumnAtIndex(1).getAsMat();
                AnkleAngle=StateDataTable.getDependentColumnAtIndex(2).getAsMat();
                BiodexAngle=StateDataTable.getDependentColumnAtIndex(3).getAsMat();
                kneelabel=string(StateDataTable.getColumnLabel(1));
                kneespeedlabel=strrep(kneelabel,'value','speed');
                anklelabe=string(StateDataTable.getColumnLabel(2));
                anklespeedlabel=strrep(anklelabe,'value','speed');
                timejav=StateDataTable.getIndependentColumn();
                r=StateDataTable.getNumRows();
                for tt=1:r
                    time(tt,1)=double(timejav.get(tt-1));
                end
                BiodexSpeed=diff(BiodexAngle)./diff(time);
                BiodexSpeed=[BiodexSpeed ;BiodexSpeed(1)];
                Biodexcombied=[BiodexAngle BiodexSpeed];
                [nrow,ncol]=size(Biodexcombied); 
                Data.(combinedname).state=Biodexcombied;

                %%% New Table
                Newstate=TimeSeriesTable();
                %%% Setup Lables
                collabels =  StdVectorString();
                if contains(Joints(T1),"Knee")
                    collabels.add(kneelabel);
                    collabels.add(kneespeedlabel);
                    Etimeindx=StateDataTable.getNumRows();
                else
                    collabels.add(anklelabe);
                    collabels.add(anklespeedlabel);
                    Etimeindx=find(BiodexAngle>= Data.maxpanlt);
                end
                
                Newstate.setColumnLabels(collabels);
                %%% Setup Data
                ncol=double(collabels.size());
                row = RowVector(ncol, 0);
                
                for iRow=1:nrow
                    row.set(0,Data.(combinedname).state(iRow,1));
                    row.set(1,Data.(combinedname).state(iRow,2));
                    %                     row.set(2,Data.(combinedname).Act(iRow,1));
                    Newstate.appendRow(iRow-1, row);
                end

                %%% setup time 
                timeColumn = Newstate.getIndependentColumn();
                for i = 1 : nrow
                    timeColumn.set(i-1, time(i));
                end
                %%% setup Keys
                Kyes=StateDataTable.getTableMetaDataKeys();
                for i=1:1:Kyes.size()
                    if contains(string(Kyes.get(i-1)),"nColumns")
                        Newstate.addTableMetaDataString(Kyes.get(i-1),string(ncol+1));
                    elseif contains(string(Kyes.get(i-1)),"nRows")
                        Newstate.addTableMetaDataString(Kyes.get(i-1),string(nrow));
                    else
                        Keval=StateDataTable.getTableMetaDataString(Kyes.get(i-1));
                        Newstate.addTableMetaDataString(Kyes.get(i-1),Keval);
                    end
                end



                   
%                 %%% definning Biodex as ref motion
%                 for tt=1:3
%                     StateDataTable.removeColumnAtIndex(0)
%                 end
%                 StateDataTable.removeTableMetaDataKey('nColumns')
%                 StateDataTable.addTableMetaDataString('nColumns','2')
                %                 Data.(filename).Hipangle=mean(HipAngle);
                %                 Meanknee=mean(KneeAngle);
                %                 if Meanknee< 0 Meanknee=0; end
                %                 Data.(filename).Kneeangle= 1.57;
                %                 Data.(filename).Ankleangle=mean(AnkleAngle);
                ControlDataTable=TableProcessor(Data.(filename).RefControlpath).process;
                StateSolutionTable=Newstate;
                ControlSolutionTable=ControlDataTable;
                Data.(filename).Stime=double(StateSolutionTable.getIndependentColumn().get(0));
                Data.(filename).Etime=double(StateSolutionTable.getIndependentColumn().get(Etimeindx(end)-1));
                [kneeTrackingSolution]=TorqueSimulation(StateSolutionTable,osimmodel,filename,Data);
                [kneeTrackingParamSolution]=ParameterEstimation(StateSolutionTable,ControlSolutionTable,osimmodel,combinedname,filename,Data);
                osimmodel=changemodelproperty(osimmodel,filename,Data,1);
                qe=qe+1;

            end
        end
    end
    fprintf('%s is done \n',SubjectNumber(S));
end
diary off
% Logger.removeSink(myLog);
save([txtBasepath '\SimData.mat'],'Data');

% Plotting()
function [data]= addingleg(data,whichleg)
for y=1:length(data)
    data(y)=append(data(y),"_",whichleg);
end
end