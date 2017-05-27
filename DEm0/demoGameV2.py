import pygame
import random

# Screen settings
res_x = 3840
res_y = 2160

# Setup for pygame
pygame.init()
screen = pygame.display.set_mode((res_x,res_y), pygame.FULLSCREEN)
myfont = pygame.font.SysFont("centurygothic", 128)
infoFont = pygame.font.SysFont("centurygothic", 64)

# Colors
white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)

# Variables needed for the game
round = 1
dotCount = 0
score = 0
done = False
time = 60
pygame.time.set_timer(pygame.USEREVENT + 1, 1000)
blank = True

while not done:
    # Update dot size based on round
    if round == 1:
        radius = 100
        diameter = 200
    elif round == 2:
        radius = 76
        diameter = 152
    else:
        radius = 50
        diameter = 100

    # Get mouse position
    mouse = pygame.mouse.get_pos()

    # Draw dot
    if blank == True and score < 15:
        x = random.randint(radius, res_x - diameter)
        y = random.randint(80 + diameter, res_y - diameter)
        pygame.draw.circle(screen, white, (x, y), radius)
        blank = False

    # Event loop
    for event in pygame.event.get():
        if event.type == pygame.USEREVENT + 1:
            time -= 1
        if event.type == pygame.QUIT:
            done = True
        if ((mouse[0] - x)**2 + (mouse[1] - y)**2 <= radius**2) and event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and dotCount < 5:
            screen.fill((black))
            dotCount += 1
            score += 1
            blank = True
        if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE and dotCount == 5 and round < 3:
            screen.fill((black))
            blank = True
            dotCount = 0
            round += 1

    # Prompt a double swipe gesture
    if dotCount == 5 and blank == True and round < 3:
        screen.fill((black))
        label = myfont.render("Do a double swipe!", 1, white)
        label_center = label.get_rect(center=(res_x/2, res_y/2))
        screen.blit(label, label_center)
        blank = False

    # End game if success
    if score == 15:
        screen.fill((black))
        label = myfont.render("You Passed!", 1, white)
        label_center = label.get_rect(center=(res_x / 2, res_y / 2))
        screen.blit(label, label_center)

    # Logic for displaying time and score
    if score < 15:
        if time <= 0:
            screen.fill((red))
            label = myfont.render("RIP", 1, white)
            label_center = label.get_rect(center=(res_x / 2, res_y / 2))
            screen.blit(label, label_center)
        if time > 0:
            pygame.draw.rect(screen, black, pygame.Rect(0, 0, res_x, 80))
            timeLabel = infoFont.render(time.__str__(), 1, white)
            screen.blit(timeLabel, (0, 0))

    # Display score
    scoreLabel = infoFont.render("Score: " + score.__str__(), 1, white)
    scoreLabel_topRight = scoreLabel.get_rect(topright=(res_x, 0))
    screen.blit(scoreLabel, scoreLabel_topRight)

    pygame.display.flip()