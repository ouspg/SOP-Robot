
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

    fms1 = open('{}_{}.scp'.format(speaker_files[0], 'fm'), 'w')

    ffs1 = open('{}_{}.scp'.format(speaker_files[0], 'ff'), 'w')

    mms1 = open('{}_{}.scp'.format(speaker_files[0], 'mm'), 'w')
    
    i = 0
    for g in grouped:
        g1 = spk1[i:i+g[1]]
        print(g[0])
        if g[0] == 'fm' or g[0] == 'mf':
            fms1.writelines(g1)
        elif g[0] == 'ff':
            ffs1.writelines(g1)
        elif g[0] == 'mm':
            mms1.writelines(g1)

        i += g[1]
    print(i)

