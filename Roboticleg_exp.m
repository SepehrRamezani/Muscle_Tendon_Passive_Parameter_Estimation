clear all
close all
import org.opensim.modeling.*;
Trc_path=['C:\MyCloud\OneDriveUcf\Real\Simulation\Passive_Parameter_prediction\Robotic_Leg\'];
listing = dir(Trc_path);
plotfalg=0;
IKflag=1;
Mnames=[];
Tnames=[];

% myLog = JavaLogSink();
% Logger.addSink(myLog);

runver="E5";
% SubjectNumber=["06","07","08","09","10","11","12","13","14","15"];
SubjectNumber=["10"];

Data.stiffness=[420.7,569.3,520.44193,391.1];
SubjectNumber=append("T0",SubjectNumber);
Data.joints=["ground_pelvis","Hip_Joint","Knee_Joint","Ankle_Joint"];
Data.Weldjoints=["ground_pelvis","Hip_Joint","Ankle_Joint"];
Data.bodies=["Hip_Bone","Thigh","Shank","Foot"];
Data.maxpanlt=-deg2rad(15);
Data.DeGrooteflage=1;
Data.TorqueSolverinterval=40;
Data.ParamSolverinterval=50;

Data.optForce=1;
psname=["Knee90Trial0","Knee45Trial0","Knee0Trial0"];
for S=length(psname)
    combinedname=psname(S);
    Data.(combinedname).ModelPath=fullfile(Trc_path,'Model',append(combinedname,runver,'.osim'));
    Data.(combinedname).RefModelpath=fullfile(Trc_path,'Model','FinalModel.osim');
    Data.(combinedname).RefStatepath=fullfile(Trc_path,'Data',append(combinedname,'_IK.mot'));
    Data.(combinedname).RefControlpath=fullfile(Trc_path,'Data',append(combinedname,'_Torque.sto'));
    Data.(combinedname).TorqeSimulPath=fullfile(Trc_path,'Result',append(combinedname,'_TorqueOpt',runver,'.sto'));
    Data.(combinedname).ParmSimulPath=fullfile(Trc_path,'Result',append(combinedname,'_ParamOpt',runver,'.sto'));
    Data.(combinedname).Hipangle=deg2rad(0);
    Data.(combinedname).Kneeangle=deg2rad(90);
    Data.(combinedname).Ankleangle=0;
    Data.ActiveCoordinates=["Knee"];
    Data.ActiveAct=["VASINT","RECFEM","BFLH","BFSH"];
%     Data.ActiveAct=["RECFEM","BFLH"]
    Refmmodel = Model(Data.(combinedname).RefModelpath);
%     osimmodel = Model(Data.(combinedname).ModelPath);
    osimmodel=Robotic_Modelcreator(combinedname,Data,Refmmodel);

    %% processing data
    StateDataTable=TableProcessor(Data.(combinedname).RefStatepath).process;
    r=StateDataTable.getNumRows();
    c=StateDataTable.getNumColumns();
    %     k=0;
    %     for i=0:c-1
    %         clname=StateDataTable.getColumnLabel(k);
    %         if ~any(contains(Data.ActiveCoordinates,string(clname)))
    %             StateDataTable.removeColumnAtIndex(k)
    %             %             c=c-1;
    %         else
    %             k=k+1;
    %         end
    %     end
    %     StateDataTable.removeTableMetaDataKey('nColumns')
    %     StateDataTable.addTableMetaDataString('nColumns','3')
    Newstate=TimeSeriesTable();


    %%% getting data
    HipAngle=StateDataTable.getDependentColumnAtIndex(6).getAsMat();
%     KneeAngle=StateDataTable.getDependentColumnAtIndex(7).getAsMat();
    KneeAngle=(90:(0-90)/(r-1):0)';
    timejav=StateDataTable.getIndependentColumn();
    for tt=1:r
        time(tt,1)=double(timejav.get(tt-1));
    end
    %%% filtering
    [bb,aa] = butter(1, 0.1,'low');
    Hipfilt=filtfilt(bb,aa,HipAngle);
    Kneefilt=filtfilt(bb,aa,KneeAngle);
    HipSpeed=diff(Hipfilt)./diff(time);
    KneeSpeed=diff(Kneefilt)./diff(time);
    NewData=[Hipfilt(1:end-1),HipSpeed,Kneefilt(1:end-1),KneeSpeed];
    newtime=time(1):(time(end-1)-time(1))/300:time(end-1);
    NewDatalowsamp=interp1(time(1:end-1),NewData, newtime);
    Data.(combinedname).Hip=NewDatalowsamp(:,[1,2]);
    Data.(combinedname).Knee=NewDatalowsamp(:,[3,4]);
    [nrow,ncol]=size(NewDatalowsamp);
    collabels =  StdVectorString();
    
    for i=1:length(Data.ActiveCoordinates)
        corrval=append("/jointset/",Data.ActiveCoordinates(i),"_Joint/",Data.ActiveCoordinates(i),"/value");
        corrspval=append("/jointset/",Data.ActiveCoordinates(i),"_Joint/",Data.ActiveCoordinates(i),"/speed");
        collabels.add(corrval);
        collabels.add(corrspval);
    end
    Newstate.setColumnLabels(collabels);
    ncol=double(collabels.size());
    row = RowVector(ncol, 0);
    for iRow=1:nrow
        for iCol = 1 :length(Data.ActiveCoordinates)
            row.set(iCol-1,Data.(combinedname).(Data.ActiveCoordinates(iCol))(iRow,iCol));
            row.set(iCol,Data.(combinedname).(Data.ActiveCoordinates(iCol))(iRow,iCol+1));
        end
        Newstate.appendRow(iRow-1, row);
    end

    timeColumn = Newstate.getIndependentColumn();
    for i = 1 : nrow
        timeColumn.set(i-1, newtime(i));
    end
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

    %%
    Data.(combinedname).Stime=newtime(1);
    Data.(combinedname).Etime=26;

%     [TqkneeTrackingSolution]=Roboticleg_TorqueSimulation(Newstate,osimmodel,combinedname,Data);

    TorqStateDataTable=TableProcessor(Data.(combinedname).TorqeSimulPath).process;
    Torqtimjav=TorqStateDataTable.getIndependentColumn();
    for tt=1:Torqtimjav.size()
        TorqOpttime(tt,1)=double(Torqtimjav.get(tt-1));
    end

    [ParkneeTrackingSolution]=Roboticleg_ParamOpt(TorqStateDataTable,osimmodel,combinedname,Data);

    ParamStateDataTable=TableProcessor(Data.(combinedname).ParmSimulPath).process;
    paramtimjav=ParamStateDataTable.getIndependentColumn();
    for tt=1:paramtimjav.size()
        ParamOpttime(tt,1)=double(paramtimjav.get(tt-1));
    end
    plot(newtime,deg2rad(Data.(combinedname).Knee(:,1)),TorqOpttime,TorqStateDataTable.getDependentColumnAtIndex(0).getAsMat());
    hold on
    plot(ParamOpttime,ParamStateDataTable.getDependentColumnAtIndex(0).getAsMat());
    hold off
    legend('exp','torque','param')
    figure 
    plot(TorqOpttime,TorqStateDataTable.getDependentColumnAtIndex(2).getAsMat());
    hold on
    plot(ParamOpttime,ParamStateDataTable.getDependentColumnAtIndex(2).getAsMat());
    legend('torque','param')

end
