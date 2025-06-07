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

function s = imul_str(val)
	s = "";
	if val != 1
		s = sprintf("%d*", val);
	end
end

function fcv = gen_fcv(Re, Im, tstr)
	n = length(Re);
	fcv = num2str(Re(1));
	if nargin() < 3
		tstr = "x";
	end
	for k = 2:n
		cstr = sprintf(" + %f*cos(%s%s)", Re(k), imul_str(k-1), tstr);
		fcv = strcat(fcv, cstr);
	end
	for k = 2:n
		sstr = sprintf(" + %f*sin(%s%s)", Im(k), imul_str(k-1), tstr);
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

	printf("Processing %d channels x %d frames.\n", nch, nfrm);
	fflush(stdout);
	fout = fopen("_expr.hs", "w");
	fprintf(fout, "chblockbegin\n");
	fprintf(fout, "frange 0 %d\n", nfrm-1);
	animPath = "/obj/ANIM";
	for chNo = 1:nch
		chName = anm_ch_name(anim, chNo);
		ch = anm_ch_data(anim, anm_ch_idx(anim, chName));
		norg = length(ch);
		if mod(norg, 2)
			% odd -> even
			ch = [ch, ch(1)]; # loop
		end
		nlen = length(chName);
		nodeName = chName(1:nlen-3);
		paramName = chName(nlen-1:nlen);
		fprintf(fout, "chadd -f 0 0 %s/%s %s\n", animPath, nodeName, paramName);
		n = length(ch);
		[Re, Im] = RDFT(ch);
		nfull = length(Re);
		inz = find(quantize(Re) != 0);
		if length(inz) == 0
			fprintf(fout, "# %s constant\n", chName);
			#fprintf(fout, "chkey -f 0 -v %f %s/%s/%s\n", ch(1), animPath, nodeName, paramName);
			fprintf(fout, "chkey -f 0 -F '%f' %s/%s/%s\n", ch(1), animPath, nodeName, paramName);
		else
			ncut = max(max(inz), 8);
			ncut = min(ncut, nfull - fix(nfull/3));
			#ncut = 9;#############################
			Re(ncut+1:nfull) = 0;
			Im(ncut+1:nfull) = 0;
			th = sprintf("((abs($FF%s%d)/%d)*360)", "%", norg, norg);
			expr = gen_fcv(Re(1:ncut), Im(1:ncut), th);
			fprintf(fout, "# %s, ncut=%d\n", chName, ncut);
			fprintf(fout, "chkey -f 0 -F '%s' %s/%s/%s\n", expr, animPath, nodeName, paramName);
		end
		fprintf(fout, "\n");
		printf(".");
		fflush(stdout);
	end
	printf("\n");
	fprintf(fout, "chblockend\n");
	fclose(fout);
	printf("Done.\n");
end

anim_proc_main();
