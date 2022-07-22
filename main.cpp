#include <iostream>
#include <SDL2/SDL.h>

#define WINDOW_WIDTH 400
#define WINDOW_HEIGHT 300
#define WIDTH 128
#define HEIGHT 120

SDL_Rect SrcR;
SDL_Rect DestR;
void MOD_SDL_Init_Rect(SDL_Rect *srcrect, SDL_Rect *dstrect) {
    srcrect->x = 0;
    srcrect->y = 0;
    srcrect->w = WIDTH;
    srcrect->h = HEIGHT;

    dstrect->x = WINDOW_WIDTH / 2 - WIDTH / 2;
    dstrect->y = WINDOW_HEIGHT / 2 - HEIGHT / 2;
    dstrect->w = WIDTH;
    dstrect->h = HEIGHT;
}

int main(int argc, char *argv[]) {
    SDL_Init(SDL_INIT_EVERYTHING);
    MOD_SDL_Init_Rect(&SrcR, &DestR);

    SDL_Window *window = SDL_CreateWindow("Simulator GUI", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == NULL) {
        std::cout << "Could not create window: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);
    if (renderer == NULL) {
        std::cout << "Could not create renderer" << std::endl;
        return 4;
    }
    SDL_SetRenderDrawColor(renderer, 0xCC, 0xCC, 0xCC, 0xFF);

    int len = WIDTH * HEIGHT * 3;
    char* pixels = new char[len];
    for (int i = 0; i < len; i++) {
        if (i % 3 == 1)
            pixels[i] = 255;
        else
            pixels[i] = 0;        
    }
    SDL_Surface *surface = SDL_CreateRGBSurfaceFrom((void*)pixels, WIDTH, HEIGHT, 8*3, HEIGHT*3, 0xFF0000, 0x00FF00, 0x0000FF, 0x000000);
    if (surface == NULL) {
        std::cout << "Error loading image: " << SDL_GetError() << std::endl;
        return 5;
    }

    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    if (texture == NULL) {
        std::cout << "Error creating texture" << std::endl;
        return 6;
    }

    for (int i = 0; i < 3; i++) {
        DestR.x += 20;

        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, &SrcR, &DestR);
        SDL_RenderPresent(renderer);
    
        SDL_Delay(500);
    }

    SDL_Event windowEvent;
    while (true) {
        if (SDL_PollEvent(&windowEvent)) {
            if (windowEvent.type == SDL_QUIT) break;
        }
    }
    SDL_FreeSurface(surface);
    SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return EXIT_SUCCESS;
}
