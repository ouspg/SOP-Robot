
genders = {}
# scp files containing speaker metadata
speaker_files = ['']

def getGenders(speakers, pairs):

    with open(speakers) as s:
        lines = s.readlines()

    for line in lines:
        line = line.strip().split('|')
        genders[int(line[0])] = line[1].strip()

    with open(pairs) as p:
        lines = p.readlines()

    grouped = []

    sum = 0
    prev = lines[0].split()
    for pair in lines:
        pair = pair.split()

        if pair[0] != prev[0]:
            grouped.append(('{}{}'.format(genders[int(prev[0])].lower(), genders[int(prev[1])].lower()).strip(), sum))
            sum = 0

        prev = pair
        sum += 1

    with open('{}.scp'.format(speaker_files[0])) as s:
        spk1 = s.readlines()
    
    with open('{}.scp'.format(speaker_files[1])) as s:
        spk2 = s.readlines()

    with open('{}.scp'.format(speaker_files[2])) as s:
        spk1_ev = s.readlines()

    with open('{}.scp'.format(speaker_files[3])) as s:
        spk2_ev = s.readlines()

    fms1 = open('{}_{}.scp'.format(speaker_files[0], 'fm'), 'w')
    fms2 = open('{}_{}.scp'.format(speaker_files[1], 'fm'), 'w')
    fms1_ev = open('{}_{}.scp'.format(speaker_files[2], 'fm'), 'w')
    fms2_ev = open('{}_{}.scp'.format(speaker_files[3], 'fm'), 'w')

    ffs1 = open('{}_{}.scp'.format(speaker_files[0], 'ff'), 'w')
    ffs2 = open('{}_{}.scp'.format(speaker_files[1], 'ff'), 'w')
    ffs1_ev = open('{}_{}.scp'.format(speaker_files[2], 'ff'), 'w')
    ffs2_ev = open('{}_{}.scp'.format(speaker_files[3], 'ff'), 'w')

    mms1 = open('{}_{}.scp'.format(speaker_files[0], 'mm'), 'w')
    mms2 = open('{}_{}.scp'.format(speaker_files[1], 'mm'), 'w')
    mms1_ev = open('{}_{}.scp'.format(speaker_files[2], 'mm'), 'w')
    mms2_ev = open('{}_{}.scp'.format(speaker_files[3], 'mm'), 'w')
    
    i = 0
    for g in grouped:
        g1 = spk1[i:i+g[1]]
        g2 = spk2[i:i+g[1]]
        g1_ev = spk1_ev[i:i+g[1]]
        g2_ev = spk2_ev[i:i+g[1]]
        print(g[0])
        if g[0] == 'fm' or g[0] == 'mf':
            fms1.writelines(g1)
            fms2.writelines(g2)
            fms1_ev.writelines(g1_ev)
            fms2_ev.writelines(g2_ev)
        elif g[0] == 'ff':
            ffs1.writelines(g1)
            ffs2.writelines(g2)
            ffs1_ev.writelines(g1_ev)
            ffs2_ev.writelines(g2_ev)
        elif g[0] == 'mm':
            mms1.writelines(g1)
            mms2.writelines(g2)
            mms1_ev.writelines(g1_ev)
            mms2_ev.writelines(g2_ev)

        i += g[1]
    print(i)

