function fig=multiplotting(fig,Ylable,XLable,thetitle,tendod_slack,newcolors,MarkerSize)
numfigs=length(Ylable);
title(fig,thetitle);
for m=1:numfigs
    nexttile
    for p=1:3
        ty(p)=mean(tendod_slack(p:p+2,m));
        terrpo(p)=max(tendod_slack(p:p+2,m))-ty(p);
        terrneg(p)=ty(p)-min(tendod_slack(p:p+2,m));
    end
    X = categorical(XLable);
    %         plot(X,[tendod_slack(1:3,m),tendod_slack(4:6,m),tendod_slack(7:9,m)],'.','MarkerSize',MarkerSize)
    e=errorbar(X,ty,terrneg,terrpo);
    e.Marker = '*';
    e.MarkerSize = MarkerSize;
    if m>size(newcolors)
        color=m-7;
    else
        color=m;
    end
    e.Color = newcolors{color};
    e.CapSize = 15;
    ylabel (Ylable(m));
end
end