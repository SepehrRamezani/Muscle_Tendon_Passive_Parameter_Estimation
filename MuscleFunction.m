epsilonM=0.2;
kPE=10;
LMhat=0:0.01:2;
FPEhat=(exp(kPE.*(LMhat-1)./epsilonM)-1)/(exp(kPE)-1);
hold on
plot(LMhat,FPEhat);