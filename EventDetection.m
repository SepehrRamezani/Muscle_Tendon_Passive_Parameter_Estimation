function Events = EventDetection(DStime,Data,Threshold)

    [bb,aa] = butter(4, 0.2,'low');
    DataFiltered=filtfilt(bb,aa,Data);
    normDataFiltered=DataFiltered/max(DataFiltered);
    Velocity=abs(diff(normDataFiltered)/DStime);
    Stimeindx=find(Velocity<=Threshold);
    selecttime=find(diff(Stimeindx)>1);
    ConstantTime=Stimeindx(1:selecttime(1)-1);
    Etimeindx=find(Velocity > Threshold);
    selecttime=find(diff(Etimeindx)>1);
    if selecttime
        EventEtime=Etimeindx(1:selecttime(1)-1);
    else
        EventEtime=Etimeindx;
    end
    Events.ConstantTime=ConstantTime;
    Events.EventEtime=EventEtime;
    
    
end