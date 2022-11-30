function [Gdata,Gheader]= ReshapingData(data,DStime)
Gdata=[0];
for ii=2:2:length(data.textdata)
    kk=1;
    tflage=0;
    ww=0;
    for jj=1:length(data.data)%# of data points in a given trial
        if ~isnan(data.data(jj,ii))&&data.data(jj,ii)~=0
            kk=jj;  %finding zero data at the end of each chanel
            tflage=1;
        elseif tflage==0
            ww=jj;  %finding zero data at the begining
        end
    end
    ts(ii/2)=data.data(ww+1,ii-1); % first time of first chanel to set as final time for every other channel.
    te(ii/2)=data.data(kk,ii-1); % final time of first chanel to set as final time for every other channel.
end
%%             finding the highest starting
tsfinal=max(ts);
tefinal=min(te);
interpolatetime=tsfinal:DStime:tefinal;
for ii=2:2:length(data.textdata)
    
    starttimeinx=find(data.data(:,ii-1)>= tsfinal & data.data(:,ii-1)<=tefinal);
    y=interp1(data.data(starttimeinx,ii-1),data.data(starttimeinx,ii),interpolatetime,'linear','extrap'); %Interpolates data to match sampling time to desierd sampling time
    
    b=y';
    %                 ends(ii)=length(b);
    if (size(Gdata(:,1)) == 1) %recombines data into a matrix padded with NaN
        Gdata = [interpolatetime' b];
    else
        Gdata = [Gdata b];
    end
    
end

Gheader=["time" data.textdata(2:2:end)];

end