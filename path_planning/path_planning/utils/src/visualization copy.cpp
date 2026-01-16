#include "visualization.hpp"

#include <iostream>


Visualization::Visualization(const std::string name) {
    name_ = name;
}


Visualization::~Visualization() {
    TTF_CloseFont(font);
    if (bg_texture) SDL_DestroyTexture(bg_texture);
    IMG_Quit();
    TTF_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}


bool Visualization::init() {
    // Initialize SDL2
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return false;
    }

    // Initialize TTF
    TTF_Init();

    // Initialize IMG
    IMG_Init(IMG_INIT_PNG);  // Optional

    window = SDL_CreateWindow(
        name_.data(),
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN
    );
    if (!window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return false;
    }

    font = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24);  // TODO Replace with actual font path

    return true;
}


void Visualization::visualizeGraph(Graph graph) {
    // Define node positions (hardcoded for 3x3 layout)
    std::vector<SDL_Point> positions = {
        {200, 100}, {400, 100}, {600, 100},  // Row 0: 0,1,2
        {200, 300}, {400, 300}, {600, 300},  // Row 1: 3,4,5
        {200, 500}, {400, 500}, {600, 500}   // Row 2: 6,7,8
    };
    const int NODE_RADIUS = 20;

    // Optional: Load background image
    // bg_texture = IMG_LoadTexture(renderer, "path/to/background.png");

    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
        }

        SDL_SetRenderDrawColor(renderer, 20, 20, 40, 255);  // Dark background
        SDL_RenderClear(renderer);

        // Optional: Draw background
        if (bg_texture) {
            SDL_RenderCopy(renderer, bg_texture, NULL, NULL);
        }

        // Draw edges (using SDL2_gfx for thickness)
        for (size_t i = 0; i < graph.size(); ++i) {
            for (int j : graph[i]) {
                if (j > i) {  // Avoid duplicate lines in undirected graph
                    thickLineRGBA(renderer,
                                positions[i].x, positions[i].y,
                                positions[j].x, positions[j].y,
                                4,  // Thickness
                                255, 255, 255, 255);  // White
                }
            }
        }

        // Draw nodes (circles with SDL2_gfx)
        for (size_t i = 0; i < positions.size(); ++i) {
            filledCircleRGBA(renderer, positions[i].x, positions[i].y, NODE_RADIUS,
                            100, 100, 255, 255);  // Blue fill
            circleRGBA(renderer, positions[i].x, positions[i].y, NODE_RADIUS,
                    255, 255, 255, 255);  // White outline
        }

        // Draw labels (text with SDL2_ttf)
        SDL_Color text_color = {255, 255, 255, 255};  // White
        for (size_t i = 0; i < positions.size(); ++i) {
            std::string label = std::to_string(i);
            SDL_Surface* text_surface = TTF_RenderText_Solid(font, label.c_str(), text_color);
            SDL_Texture* text_texture = SDL_CreateTextureFromSurface(renderer, text_surface);

            int tex_w = 0, tex_h = 0;
            SDL_QueryTexture(text_texture, NULL, NULL, &tex_w, &tex_h);

            SDL_Rect dst_rect = {positions[i].x - tex_w / 2,
                                positions[i].y - tex_h / 2,
                                tex_w, tex_h};
            SDL_RenderCopy(renderer, text_texture, NULL, &dst_rect);

            SDL_FreeSurface(text_surface);
            SDL_DestroyTexture(text_texture);
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(16);  // ~60 FPS
    }
}











    // void visualizeGrid() {
    //     bool running = true;
    //     SDL_Event event;

    //     while (running) {
    //         while (SDL_PollEvent(&event)) {
    //             if (event.type == SDL_QUIT) running = false;
    //         }

    //         // Set background
    //         SDL_SetRenderDrawColor(renderer, 20, 20, 40, 255);  // Dark background
    //         SDL_RenderClear(renderer);

    //         // Draw grid cells
    //         drawGrid();

    //         // Draw path highlight
    //         drawPath();

    //         // Or simpler: fill path cells
    //         // SDL_SetRenderDrawColor(renderer, 255, 255, 0, 255);  // Yellow
    //         // for (auto& p : path) { ... compute rect, SDL_RenderFillRect(...) }

    //         SDL_RenderPresent(renderer);

    //         SDL_Delay(16);  // ~60 FPS
    //     }
    // }


    // void drawGrid() {
    //     for (int row = 0; row < GRID_ROWS; ++row) {
    //         for (int col = 0; col < GRID_COLS; ++col) {
    //             SDL_Rect cell;
    //             cell.x = col * CELL_SIZE;
    //             cell.y = row * CELL_SIZE;
    //             cell.w = cell.h = CELL_SIZE;

    //             if (/* cell is obstacle */) {
    //                 SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);  // Dark gray
    //             } else {
    //                 SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);  // Light gray free
    //             }

    //             // Special for start/goal
    //             if (/* is start */) SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
    //             if (/* is goal */) SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);

    //             SDL_RenderFillRect(renderer, &cell);

    //             // Optional grid lines
    //             SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
    //             SDL_RenderDrawRect(renderer, &cell);
    //         }
    //     }
    // }


    // void drawPath() {
    //     SDL_SetRenderDrawColor(renderer, 0, 255, 255, 255);  // Cyan thick path
    //     for (size_t i = 0; i + 1 < path.size(); ++i) {
    //         int x1 = path[i].second * CELL_SIZE + CELL_SIZE / 2;
    //         int y1 = path[i].first * CELL_SIZE + CELL_SIZE / 2;
    //         int x2 = path[i+1].second * CELL_SIZE + CELL_SIZE / 2;
    //         int y2 = path[i+1].first * CELL_SIZE + CELL_SIZE / 2;

    //         // Draw thick line by multiple calls (e.g., 5px thick)
    //         for (int offset = -2; offset <= 2; ++offset) {
    //             SDL_RenderDrawLine(renderer, x1 + offset, y1, x2 + offset, y2);
    //             SDL_RenderDrawLine(renderer, x1, y1 + offset, x2, y2 + offset);
    //         }
    //     }
    // }


