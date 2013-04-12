import win32api


if False:
    loops = 0
    max = 30000
    keycodes = [ord('A'), ord('D'), ord('W'), ord('S'), ord('Q'), ord('E'), ord('H'), ord('N'), ord('J'), ord('L'), ord('I'), ord('K')]
    while True:
        presses = [0.] * len(keycodes)
        for i in xrange(max):
            for j in xrange(len(keycodes)):
                if win32api.GetAsyncKeyState(keycodes[j]):
                    presses[j] += 1
        print ''
        print '        '.join(['A', 'D' , 'W', 'S', 'Q', 'E', 'H', 'N', 'J', 'L', 'I', 'K'])
        print '     '.join(["%.2f" % (i/max) for i in presses])