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
        pixels[i] = 0;
    }

    SDL_Texture *texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);

    SDL_Event windowEvent;
    int counter = 0;
    while (true) {
        if (SDL_PollEvent(&windowEvent)) {
            if (windowEvent.type == SDL_QUIT) break;
        }

        pixels[counter] = 255 - pixels[counter];
        pixels[counter+1] = 255 - pixels[counter+1];
        pixels[counter+2] = 255 - pixels[counter+2];

        SDL_RenderClear(renderer);
        SDL_UpdateTexture(texture, NULL, pixels, HEIGHT*3);
        SDL_RenderCopy(renderer, texture, &SrcR, &DestR);
        SDL_RenderPresent(renderer);
    
        counter += 3;
        SDL_Delay(1);
    }
    SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return EXIT_SUCCESS;
}
