import pygame
import random

pygame.init()
screen = pygame.display.set_mode((1920,1080), pygame.FULLSCREEN)
myfont = pygame.font.SysFont("centurygothic", 128)
timeFont = pygame.font.SysFont("centurygothic", 64)
done = False
white = (255,255,255)
red = (255, 0, 0)
black = (0,0,0)
blank = False
count = 0
text = False
end = False
finish = False
x = random.randint(35, 1850)
y = random.randint(150, 1010)
pygame.draw.circle(screen, white, (x, y), 35)
time = 60 #time in seconds
pygame.time.set_timer(pygame.USEREVENT + 1, 1000)
while not done:
    mouse = pygame.mouse.get_pos()
    for event in pygame.event.get():
        if event.type == pygame.USEREVENT + 1:
            time -= 1
        if event.type == pygame.QUIT:
            done = True
        if ((mouse[0] - x)**2 + (mouse[1] - y)**2 <= 1225) and event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            count += 1
            blank = True
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE and text == True:
            screen.fill((0,0,0))
            x = random.randint(35, 1850)
            y = random.randint(150, 1010)
            pygame.draw.circle(screen, white, (x, y), 35)
            text = False
    if (count < 5 and blank == True):
        pygame.draw.circle(screen, black, (x, y), 35)
        x = random.randint(35, 1850)
        y = random.randint(150, 1010)
        pygame.draw.circle(screen, white, (x, y), 35)
        blank = False
    if count == 5 and blank == True and end == False:
        pygame.draw.circle(screen, black, (x, y), 35)
        label = myfont.render("Do a double swipe!", 1, white)
        screen.blit(label, (400, 400))
        text = True
        count = 0
        blank = False
        end = True
    if (count == 5 and blank == True and end == True):
        pygame.draw.circle(screen, black, (x, y), 35)
        label = myfont.render("Thanks for a 4.0!", 1, white)
        screen.blit(label, (475, 400))
        finish = True
    if (time >= 0 and finish == False):
        pygame.draw.rect(screen, black, pygame.Rect(0, 0, 1920, 80))
        timeLabel = timeFont.render(time.__str__(), 1, white)
        screen.blit(timeLabel, (0, 0))
    elif finish == True:
        pygame.draw.rect(screen, black, pygame.Rect(0, 0, 1920, 80))
    elif time <= 0 and finish == False:
        screen.fill(red)
        label = myfont.render("RIP", 1, white)
        screen.blit(label, (865, 400))
    pygame.display.flip()