function Events = EventDetection(DStime,Data,Threshold,TorqueFlage)


    [bb,aa] = butter(1, 0.4,'low');
    DataFiltered=filtfilt(bb,aa,Data(:,1));
    normDataFiltered=DataFiltered/max(DataFiltered);
    Velocity=abs(diff(normDataFiltered)/DStime);
    Stimeindx=find(Velocity<=Threshold);
    selecttime=find(diff(Stimeindx)>1);
    ConstantTime=Stimeindx(1:selecttime(1)-1);
    Etimeindx=find(Velocity > Threshold);
    selecttime=find(diff(Etimeindx)>1);
    if TorqueFlage
        [maxtorque,maxIndx]=max(Data(:,2));
        EtimeindxT=find(Stimeindx> maxIndx);
        EventEtime=maxIndx:Stimeindx(EtimeindxT(1));
    else
        if selecttime
            EventEtime=Etimeindx(1:selecttime(1)-1);
        else
            EventEtime=Etimeindx;
        end
    end
    Events.ConstantTime=ConstantTime;
    Events.EventEtime=EventEtime;
    
end