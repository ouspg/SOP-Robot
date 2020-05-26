#!/usr/local/bin/python
# -*- coding: utf-8 -*-

from pydub import AudioSegment
import os

aalto_aanne = {
"vokaali" : {
	"a": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/a.wav")),
	"e": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/e.wav")),
	"i": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/i.wav")),
	"o": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/o.wav")),
	"u": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/u.wav")),
	"y": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/y.wav")),
	"å": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/o.wav")),
	"ä": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ä.wav")),
	"ö": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ö.wav"))
	},
"konsonantti" :{
	"b": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/b.wav")),
	"c": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/c.wav")), 
	"d": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/d.wav")),
	"f": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/f.wav")),
	"g": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/g.wav")), 
	"h": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/h.wav")),
	"j": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/j.wav")),
	"k": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/k.wav")),
	"l": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/l.wav")),
	"m": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/m.wav")),
	"n": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/n.wav")),
	"p": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/p.wav")),
	"q": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/k.wav")),    #suomeksi kuulostaa k:lta
	"r": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/r.wav")),
	"s": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/s.wav")),
	"t": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/t.wav")),
	"v": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/v.wav")),
	"w": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/v.wav")),
	"x": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/x.wav")),      
	"z": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/z.wav")),     
	" ": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),    #samat molemmilla, ei tarvita molemmilta omia välimerkkien ääniä
	".": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/puols.wav")),          
	",": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/puols.wav")),
	";": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/puols.wav")),
	":": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/puols.wav")),    
	"ng": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ng.wav"))
	},   
"diftongi" : {                                         #uusi, diftongit
	"ai": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ai.wav")),
	"ei": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ei.wav")),
	"oi": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/oi.wav")),
	"ui": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ui.wav")),
	"yi": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/yi.wav")),
	"äi": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/äi.wav")),
	"öi": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/öi.wav")),
	"au": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/au.wav")),
	"eu": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/eu.wav")),    
	"iu": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/iu.wav")),
	"ei": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ei.wav")),
	"ou": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ou.wav")),
	"äy": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/äy.wav")),
	"öy": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/öy.wav")),
	"iy": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/iy.wav")),
	"ey": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ey.wav")),
	"ie": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ie.wav")),
	"uo": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/uo.wav")),
        "yö": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/yö.wav"))
	},
"aakkoset" : {
	"aa": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/aa.wav")),    #tuplaäänteet aakkosissa, jos ei ole olemassa, on tyhjä
	"bb": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/bb.wav")),    
	#"cc": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")), 
	#"dd": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	"ee": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ee.wav")),
	"ff": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ff.wav")),
	#"gg": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	#"hh": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	"ii": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ii.wav")),
	#"jj": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	"kk": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/kk.wav")),
	"ll": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ll.wav")),
	"mm": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/mm.wav")),
	"nn": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/nn.wav")),
	"oo": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/oo.wav")),
	#"pp": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	"qq": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/kk.wav")),
	"rr": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/rr.wav")),
	"ss": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ss.wav")),
	#"tt": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	"uu": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/uu.wav")),
	#"vv": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	#"ww": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	#"xx": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	"yy": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/yy.wav")),
	#"zz": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/blank.wav")),
	"åå": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/oo.wav")),
	"ää": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/ää.wav")),
	"öö": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"tl_sounds/öö.wav")),
	"  ": AudioSegment.from_wav(os.path.join(os.path.dirname(__file__),"skpeak/kohinaa.wav"))
	}
} 
