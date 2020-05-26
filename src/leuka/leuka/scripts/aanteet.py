#!/usr/local/bin/python
# -*- coding: utf-8 -*-

from pydub import AudioSegment
import os

aalto_aanne = {
"vokaali" : {
	"a": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/a.wav')),
	"e": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/e.wav')),
	"i": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/i.wav')),
	"o": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/o.wav')),
	"u": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/u.wav')),
	"y": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/y.wav')),
	"å": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/o.wav')),
	"ä": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/ae.wav')),
	"ö": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/oe.wav'))
	},
"konsonantti" :{
	"b": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/b.wav')),
	"c": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/c_k.wav')), #c_k
	"d": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/d.wav')),
	"f": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/f.wav')),
	"g": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/g.wav')),
	"h": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/h.wav')),
	"j": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/j.wav')),
	"k": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/k.wav')),
	"l": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/l.wav')),
	"m": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/m.wav')),
	"n": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/n.wav')),
	"p": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/p.wav')),
	"q": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/q.wav')),
	"r": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/r.wav')),
	"s": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/s.wav')),
	"t": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/t.wav')),
	"v": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/v.wav')),
	"w": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/v.wav')),
	"x": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/x.wav')),
	"z": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/z.wav')),
	" ": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/blank.wav')),
	".": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/puols.wav')),
	",": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/puols.wav')),
	";": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/puols.wav')),
	":": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/puols.wav')),
	"ng": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/-ng-ng.wav'))
	},
"aakkoset" : {
	"aa": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/aa.wav')),
	"bb": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/bb.wav')),
	"cc": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/cc.wav')), 
	"dd": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/dd.wav')),
	"ee": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/ee.wav')),
	"ff": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/ff.wav')),
	"gg": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/gg.wav')),
	"hh": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/hh.wav')),
	"ii": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/ii.wav')),
	"jj": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/jj.wav')),
	"kk": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/kk.wav')),
	"ll": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/ll.wav')),
	"mm": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/mm.wav')),
	"nn": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/nn.wav')),
	"oo": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/oo.wav')),
	"pp": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/pp.wav')),
	"qq": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/qq.wav')),
	"rr": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/rr.wav')),
	"ss": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/ss.wav')),
	"tt": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/tt.wav')),
	"uu": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/uu.wav')),
	"vv": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/vv.wav')),
	"ww": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/vv.wav')),
	"xx": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/xx.wav')),
	"yy": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/yy.wav')),
	"zz": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/zz.wav')),
	"åå": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/oo.wav')),
	"ää": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/aeae.wav')),
	"öö": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/oeoe.wav')),
	"  ": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__), 'skpeak/kohinaa.wav'))
	}
}
