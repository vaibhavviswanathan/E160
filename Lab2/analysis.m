clear
load ForwardTest1.mat

%%
close all

times_at_locs = [0, 18, 29, 42, 58, 72, 88, 105, 123, 142, 160]';
inds = zeros(size(times_at_locs));
for i = 1:length(inds)
    [~,I] = min(abs(timejag - times_at_locs(i)));
    inds(i) = I;
end

xjag_locs = xjag(inds);
yjag_locs = yjag(inds);
ym = ym - ym(1);

subplot(2,1,1);
plot(location,[xjag_locs,xm])
subplot(2,1,2);
plot(location,[yjag_locs,ym])
grid on

figure();
plot(xjag_locs, yjag_locs, '*', xm, ym, '*')
grid on

xerror = xm - xjag_locs;
yerror = ym - yjag_locs;

figure();
plot(location,[xerror,yerror])

grid on

figure();

