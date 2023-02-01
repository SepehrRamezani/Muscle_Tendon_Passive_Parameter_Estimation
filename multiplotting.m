function fig=multiplotting(fig,Ylable,XLable,thetitle,tendod_slack,newcolors,MarkerSize,errorbarflage)
numfigs=length(Ylable);
cc=0;
for p=1:3:9
    cc=cc+1;
    ty(cc,:)=mean(tendod_slack(p:p+2,:));
    terrpo(cc,:)=max(tendod_slack(p:p+2,:))-ty(cc,:);
    terrneg(cc,:)=ty(cc,:)-min(tendod_slack(p:p+2,:));
end
for m=1:numfigs
    if m>size(newcolors)
        color=m-7;
    else
        color=m;
    end
    X = categorical(XLable);
    if(errorbarflage)
        nexttile
        title(fig,thetitle);
        %         plot(X,[tendod_slack(1:3,m),tendod_slack(4:6,m),tendod_slack(7:9,m)],'.','MarkerSize',MarkerSize)
        e=errorbar(X',ty(:,m),terrneg(:,m),terrpo(:,m));
        e.Marker = '*';
        e.MarkerSize = MarkerSize;
        e.CapSize = 15;
        e.Color = newcolors{color};
        ylabel (Ylable(m));
    else
        title(fig,thetitle);
        plot(X',ty,'.','MarkerSize',MarkerSize)
        ylabel (Ylable(m));
        
    end

   


end
end