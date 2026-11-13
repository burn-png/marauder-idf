import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from appasm import Asm, make_app
W = 24                      # 24 x 24 cells of 10 px = 240 x 240; status line below
a = Asm()
for v in ('dir', 'nx', 'ny', 'head', 'tail', 'len', 'fx', 'fy', 'score', 'tx', 'ty', 'k', 'delay', 'i', 'idx'): a.var(v)
a.var('xs', 256); a.var('ys', 256); a.var('grid', W * W)
BLACK, WHITE, GREEN, RED, CYAN, AMBER, GRAY, BLUE = range(8)

def cell(xe, ye, color, size=9):
    a.rect([*xe, 10, '*'], [*ye, 10, '*'], size, size, color)

def draw_score():
    a.rect(56, 244, 48, 8, BLACK)
    a.e(56, 244, 'score', AMBER); a.op('NUM')

def place_food():
    a.label('food_try%d' % place_food.n); n = place_food.n; place_food.n += 1
    a.set('fx', W, 'rnd'); a.set('fy', W, 'rnd')
    a.e('fy', W, '*', 'fx', '+', '@grid'); a.jnz('food_try%d' % n)
    cell(['fx'], ['fy'], RED)
place_food.n = 0

# ---------------- (re)start ----------------
a.label('restart')
a.op('CLS')
a.text(0, 244, WHITE, 'SCORE')
a.text(120, 244, GRAY, 'HOLD CENTER = EXIT')
a.set('i', 0)
a.label('clr'); a.sta('grid', [0], ['i']); a.set('i', 'i', 1, '+'); a.e('i', W * W, '<'); a.jnz('clr')
# body: (10,12) (11,12) (12,12), head at index 2
for n, x in enumerate((10, 11, 12)):
    a.sta('xs', [x], [n]); a.sta('ys', [12], [n]); a.sta('grid', [1], [12 * W + x]); cell([x], [12], GREEN)
a.set('head', 2); a.set('tail', 0); a.set('len', 3); a.set('dir', 4); a.set('score', 0)
place_food(); draw_score()

# ---------------- main loop ----------------
a.label('loop')
a.set('k', 'key')
# accept k in 1..4 unless it reverses the current direction: opposite(dir) = ((dir-1)^1)+1
a.e('k', 1, '>=', 'k', 4, '<=', '&', 'k', 'dir', 1, '-', 1, '^', 1, '+', '!=', '&'); a.jz('keep_dir')
a.set('dir', 'k')
a.label('keep_dir')
a.set('nx', 'head', '@xs', 'dir', 4, '==', '+', 'dir', 3, '==', '-')
a.set('ny', 'head', '@ys', 'dir', 2, '==', '+', 'dir', 1, '==', '-')
a.e('nx', 0, '<', 'nx', W, '>=', '|', 'ny', 0, '<', '|', 'ny', W, '>=', '|'); a.jnz('over')
a.set('idx', 'ny', W, '*', 'nx', '+')
a.e('idx', '@grid'); a.jnz('over')
# advance the head
a.set('head', 'head', 1, '+', 255, '&')
a.sta('xs', ['nx'], ['head']); a.sta('ys', ['ny'], ['head']); a.sta('grid', [1], ['idx'])
cell(['nx'], ['ny'], GREEN)
a.e('nx', 'fx', '==', 'ny', 'fy', '==', '&'); a.jz('no_eat')
a.set('score', 'score', 1, '+'); a.set('len', 'len', 1, '+')
a.e('len', 250, '>='); a.jnz('win')
place_food(); draw_score()
a.jmp('after_move')
a.label('no_eat')
a.set('tx', 'tail', '@xs'); a.set('ty', 'tail', '@ys')
a.sta('grid', [0], ['ty', W, '*', 'tx', '+'])
a.rect(['tx', 10, '*'], ['ty', 10, '*'], 10, 10, BLACK)
a.set('tail', 'tail', 1, '+', 255, '&')
a.label('after_move')
a.set('delay', 140, 'score', 3, '*', '-')
a.e('delay', 60, '<'); a.jz('delay_ok'); a.set('delay', 60)
a.label('delay_ok')
a.e('delay'); a.op('FRAME'); a.jmp('loop')

a.label('over')
a.text(84, 100, RED, 'GAME OVER')
a.text(56, 116, WHITE, 'CENTER = RESTART')
a.jmp('wait')
a.label('win')
a.text(88, 100, GREEN, 'YOU WIN!')
a.text(56, 116, WHITE, 'CENTER = RESTART')
a.label('wait')
a.e(50); a.op('FRAME')
a.set('k', 'key'); a.e('k', 5, '=='); a.jnz('restart')
a.jmp('wait')

code = a.finish()
out = sys.argv[1] if len(sys.argv) > 1 else 'SNAKE.APP'
open(out, 'w', newline='\n').write(make_app('Snake', 'Games', '1.0', 'Joystick snake. Eat red, avoid walls and yourself. Center restarts.', code))
print('snake: %d bytes of code -> %s' % (len(code), out))
