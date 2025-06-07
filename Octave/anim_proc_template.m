clear all;

function anim = anm_load()
	load "anim.mat"
	anim.name = name';
	anim.data = data';
	anim.info = info';
end

% utils

function rad = deg2rad(deg)
	rad = deg * pi/180;
end

function x = lerp(a, b, t)
	x = a + (b - a)*t;
end

function n = v_nrm(v)
	n = v / norm(v);
end

function q = q_axis_ang_n(n, ang)
	hang = ang*0.5;
	s = sin(hang);
	c = cos(hang);
	q = [s*n(1), s*n(2), s*n(3), c];
end

function q = q_axis_ang(ax, ang)
	q = q_axis_ang_n(v_nrm(ax), ang);
end

function q = q_mul(qa, qb)
	sa = qa(4);
	va = qa(1:3);
	sb = qb(4);
	vb = qb(1:3);
	s = sa*sb - dot(va, vb);
	v = cross(va, vb) + sa*vb + sb*va;
	q = [v, s];
end

function q = q_rot(rxyz, rord)
	I = eye(3);
	stk = zeros(3, 4);
	for i=1:3
		ax = rord(i)-"x"+1;
		stk(i,:) = q_axis_ang_n(I(ax,:), rxyz(ax));
	end
	q = q_mul(q_mul(stk(3,:), stk(2,:)), stk(1,:));
end

function q = q_rot_deg(rxyz, rord)
	if nargin() < 2
		rord = "xyz";
	end
	q = q_rot(deg2rad(rxyz), rord);
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

function [fno, ffrac] = anm_fno(anim, frm)
	nfrm = size(anim.data, 2);
	ff = abs(mod(frm, nfrm));
	fno = fix(ff);
	ffrac = ff - fno;
end

function val = anm_ch_eval(anim, idx, frm)
	val = 0;
	if idx > 0
		data = anim.data(idx, :);
		[f, t] = anm_fno(anim, frm);
		val = data(f+1);
		if t
			f1 = anm_fno(anim, f+1);
			v1 = data(f1+1);
			val = lerp(val, v1, t);
		end
	end
end

function name = anm_node_name(anim, inode)
	info = anim.info(inode,:);
	name = deblank(info(:,(34:end)));
end

function xord = anm_xord(anim, inode)
	info = anim.info(inode,:);
	xord = info(:,1:3);
end

function rord = anm_rord(anim, inode)
	info = anim.info(inode,:);
	rord = info(:,4:6);
end

function lst = chlst_s2i(slst)
	lst = [str2num(slst(1:3)), str2num(slst(4:6)), str2num(slst(7:9))];
end

function lst = anm_pos_chans(anim, inode)
	info = anim.info(inode,:);
	lst = chlst_s2i(info(:,7:15));
end

function lst = anm_rot_chans(anim, inode)
	info = anim.info(inode,:);
	lst = chlst_s2i(info(:,16:24));
end

function lst = anm_scl_chans(anim, inode)
	info = anim.info(inode,:);
	lst = chlst_s2i(info(:,25:33));
end

function n = anm_num_pos_ch(anim, inode)
	lst = anm_pos_chans(anim, inode);
	n = length(find(lst > 0));
end

function n = anm_num_rot_ch(anim, inode)
	lst = anm_rot_chans(anim, inode);
	n = length(find(lst > 0));
end

function n = anm_num_scl_ch(anim, inode)
	lst = anm_scl_chans(anim, inode);
	n = length(find(lst > 0));
end

function anm_print_node_info(anim)
	n = size(anim.info, 1);
	for i=1:n
		xord = anm_xord(anim, i);
		rord = anm_rord(anim, i);
		name = anm_node_name(anim, i);
		npos = anm_num_pos_ch(anim, i);
		nrot = anm_num_rot_ch(anim, i);
		nscl = anm_num_scl_ch(anim, i);
		printf("[%d] %s: %s %s #t=%d, #r=%d, #s=%d\n",
		        i, name, xord, rord,
		        npos, nrot, nscl);
	end
end

function inode = anm_node_idx(anim, name)
	n = size(anim.info, 1);
	inode = 0;
	for i=1:n
		if strcmp(anm_node_name(anim, i), name)
			inode = i;
			break;
		end
	end
end

function q = anm_node_quat(anim, inode, frm)
	lst = anm_rot_chans(anim, inode);
	rxyz = zeros(1, 3);
	for i=1:3
		cidx = lst(i);
		if cidx > 0
			rxyz(i) = anm_ch_eval(anim, cidx, frm);
		end
	end
	rord = anm_rord(anim, i);
	q = q_rot_deg(rxyz, rord);
end

function res = is_pos_ch(chName)
	prmName = substr(chName, length(chName)-1);
	res = prmName == "tx" || prmName == "ty" || prmName == "tz";
end

function res = is_rot_ch(chName)
	prmName = substr(chName, length(chName)-1);
	res = prmName == "rx" || prmName == "ry" || prmName == "rz";
end

function chDst = ch_proc(chName, chSrc)
	nfrm = length(chSrc);
	chDst = chSrc; % pass-thru
	% actual processing code goes here
	% ...
end

function anim_proc_main()
	anim = anm_load();
	[nch, nfrm] = size(anim.data);
	nnode = size(anim.info, 1);
	printf("Processing %d channels x %d frames...", nch, nfrm);
	fflush(stdout);

	%anm_print_node_info(anim);

	outData = [;];
	for chNo = 1:nch
		chName = anm_ch_name(anim, chNo);
		%printf("[%d] %s\n", chNo, chName);
		chSrc = anm_ch_data(anim, chNo);
		chDst = ch_proc(chName, chSrc);
		outData(end+1,:) = real(chDst);
	end
	printf(" done.\n");

	printf("Saving Houdini clip...");
	fflush(stdout);
	fout = fopen("anim.clip", "w");
	fprintf(fout, "{\n");
	fprintf(fout, "	rate = %d\n", 60);
	fprintf(fout, "	start = %d\n", -1);
	fprintf(fout, "	tracklength = %d\n", nfrm);
	fprintf(fout, "	tracks = %d\n", nch);
	for chNo = 1:nch
		chName = anm_ch_name(anim, chNo);
		fprintf(fout, "   {\n");
		fprintf(fout, "      name = %s/%s\n", "/obj", chName);
		fprintf(fout, "      data =");
		for fno = 1:nfrm
			val = outData(chNo, fno);
			fprintf(fout, " %f", val);
		end
		fprintf(fout, "\n");
		fprintf(fout, "   }\n");
	end
	fprintf(fout, "}\n");
	fclose(fout);
	printf(" done.\n");
end

anim_proc_main();
