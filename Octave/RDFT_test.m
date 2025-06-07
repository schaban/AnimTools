clear all;

function anim = anm_load()
	load "anim.mat"
	anim.name = name';
	anim.data = data';
	anim.info = info';
end


% anim

function name = anm_ch_name(anim, idx)
	name = deblank(anim.name(idx,:));
end

function idx = anm_ch_idx(anim, name)
	n = length(anim.name);
	idx = 0;
	for i=1:n
		if strcmp(anm_ch_name(anim, i), name)
			idx = i;
			break
		end
	end
end

function data = anm_ch_data(anim, idx)
	data = [];
	if idx > 0
		data = anim.data(idx, :);
	end
end

% 0-based index
function [fidx, ffrac] = anm_fidx(anim, frm)
	nfrm = size(anim.data, 2);
	ff = abs(mod(frm, nfrm));
	fidx = fix(ff);
	ffrac = ff - fidx;
end


% RDFT

function [Re, Im] = RDFT(ch)
	n = length(ch);
	hn = fix(n / 2);
	Re = zeros(1, hn);
	Im = zeros(1, hn);
	for k = 1:hn
		for s = 1:n
			t = 2*pi*(k-1)*(s-1)/n;
			Re(k) += ch(s)*cos(t);
			Im(k) -= ch(s)*sin(t);
		end
	end
	Re ./= hn;
	Re(1) /= 2;
	Re(hn) /= 2;
	Im = -Im ./ hn;
end

function ch = RDFT_inv(Re, Im, n)
	ch = zeros(1, n);
	hn = fix(n / 2);
	for k = 1:hn
		for s = 1:n
			t = 2*pi*(k-1)*(s-1)/n;
			ch(s) += Re(k)*cos(t) + Im(k)*sin(t);
		end
	end
end


function fcv = gen_fcv(Re, Im, tstr)
	n = length(Re);
	fcv = num2str(Re(1));
	if nargin() < 3
		tstr = "x";
	end
	for k = 2:n
		cstr = sprintf(" + %f*cos(%d*%s)", Re(k), k-1, tstr);
		fcv = strcat(fcv, cstr);
	end
	for k = 2:n
		sstr = sprintf(" + %f*sin(%d*%s)", Im(k), k-1, tstr);
		fcv = strcat(fcv, sstr);
	end
end

function q = quantize(x, qs)
	if nargin() < 2
		nx = norm(x);
		if nx
			qs = 0.1;
			if nx < 1
				qs /= 200;
			end
		else
			qs = 0.1;
		end
	end
	q = floor(abs(x) / qs) * qs .* sign(x);
end

function anim_proc_main()
	anim = anm_load();
	[nch, nfrm] = size(anim.data);
	nnode = size(anim.info, 1);

	chName = "j_Neck:ry";
#chName = "j_Hip_L:rx";
chName = "j_Spine:ry";
#chName = "j_Pelvis:ry";
#chName = "n_Center:tx";
	disp(chName);
	ch = anm_ch_data(anim, anm_ch_idx(anim, chName));
	nch = length(ch);
	mirFlg = false;
	if abs(ch(nch) - ch(1)) > (max(ch) - min(ch)) * 0.75
		ch = [ch, ch(nfrm:-1:1)]; # mirror
		disp("Mirror");
		mirFlg = true;
	elseif mod(nch, 2)
		% odd -> even
		ch = [ch, ch(1)]; # loop
		disp("odd -> even loop");
	end
	nch = length(ch);
	#plot(ch);
	[Re, Im] = RDFT(ch);
	nfull = length(Re);
	inz = find(quantize(Re) != 0);
	if length(inz) == 0
		disp("constant");
		return;
	end
	ncut = max(max(inz), 8);
	ncut = min(ncut, nfull - fix(nfull/3));
	if mirFlg
		ncut = round(ncut*0.75);
	end
	nfull
	ncut
	printf("%d -> %d\n", nfrm, ncut + (ncut-1));
	% cut
	Re(ncut+1:nfull) = 0;
	Im(ncut+1:nfull) = 0;
	%
	plot([Re',Im'], "linewidth", 1.25);
	syn = RDFT_inv(Re, Im, nch);
	#plot([ch', syn'], "linewidth", 1.0);
	plot([(ch(1:nfrm))', (syn(1:nfrm))'], "linewidth", 1.0);
	fcv = gen_fcv(Re(1:ncut), Im(1:ncut));
	if 0
		th = sprintf("((abs($FF%s%d)/%d)*360)", "%", nch, nch);
		expr = gen_fcv(Re(1:ncut), Im(1:ncut), th);
		fout = fopen("_fcv.txt", "w");
		fprintf(fout, "anim_fn(x):=%s\n", fcv);
		fprintf(fout, "%s", expr);
		fclose(fout);
	end
end

anim_proc_main();
