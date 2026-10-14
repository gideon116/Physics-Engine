#include <iostream>
#include "player.h"
#include "physics.h"
#include <SFML/Graphics.hpp>

std::array<float, 12> makeRectangle(const float w, const float h)
{
    std::array<float, 12> array = {0.f, h, 0.f, 0.f, w, h, 0.f, 0.f, w, h, w, 0.f};
    return array;
}

std::array<float, 6> makeTriangle(const float side)
{
    std::array<float, 6> array = {0.f, 0.f, side/2.f, (side/2.f) * std::sqrt(3.f), side, 0.f};
    return array;
}

std::array<float, 24> makeConcave(const float side)
{
    float sqrt3 = std::sqrt(3.f);
    float sd2 = side/2.f;
    std::array<float, 24> array =
    {
        0.f, 0.f, side, 0.f, sd2, sd2 * sqrt3,
        sd2, sd2 * sqrt3, side, 0.f, side + sd2, sd2 * sqrt3,
        0.f, 0.f, sd2, sd2 * sqrt3, -sd2, sd2 * sqrt3,
        sd2, sd2 *  sqrt3, side + sd2, sd2 * sqrt3, side, side * sqrt3,
    };
    return array;
}

int main()
{
    int frame = 0;
    int delay = 0;
    float frictionCoe = 0.1;
    float gravity = 100;

    sf::RenderWindow window(sf::VideoMode({800, 600}), "Physics yay");
    window.setFramerateLimit(60);

    float boundRight = (float)window.getSize().x;
    float boundLeft = 0;
    float boundBottom = 550;
    float boundTop = 50;

    float e = 0.6;

    // set the scene
    std::array<float, 12> ground_sky_arr = makeRectangle(800, 50);
    std::array<float, 12> left_right_arr = makeRectangle(50, 496);
    
    std::array<float, 6> sbob_arr = makeTriangle(64);
    std::array<float, 24> spat_arr = makeConcave(64);
    std::array<float, 12> sbox_arr = makeRectangle(200, 50);

    Player groundS = {"groundS", 400, 550+25, ground_sky_arr.data(), ground_sky_arr.size()/2, 2e9};
    Player skyS = {"skyS", 400, 25, ground_sky_arr.data(), ground_sky_arr.size()/2, 2e9};
    Player left = {"left", -25, 300, left_right_arr.data(), left_right_arr.size()/2, 2e9};
    Player right = {"right", 825, 300, left_right_arr.data(), left_right_arr.size()/2, 2e9};

    Player sBob = {"sBob", 50, 100, sbob_arr.data(), sbob_arr.size()/2, 2};
    Player sPat = {"sPat", 400, 100, spat_arr.data(), spat_arr.size()/2, 2};
    Player sBox = {"sBox", 200, 300, sbox_arr.data(), sbox_arr.size()/2, 4};

    std::vector<Player*> players = {&sBob, &sPat, &sBox, &groundS, &skyS, &left, &right};
    
    sBob.applyForce(-3*Pi/2, gravity);
    sPat.applyForce(-3*Pi/2, gravity);
    sBox.applyForce(-3*Pi/2, gravity);

    while (window.isOpen())
    {
        while (const std::optional event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        window.clear(sf::Color(180, 255, 180));

        // -------------------
        frame++;

        if ((!delay || (frame - delay) > 10))
        {
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Left))
            { 
                sBob.pulse(Pi, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Right))
            { 
                sBob.pulse(0, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up))
            { 
                sBob.pulse(-Pi/2, 100);
                delay = frame;
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down))
            { 
                sBob.pulse(-3*Pi/2, 100);
                delay = frame;
            }
        }

        {
            for (auto p : players)
                p->updatePos();

            for (auto p1 : players)
                for (auto p2 : players)
                    if (p1 != p2)
                        for (int i = 0; i < (p1->getNumVertices())/3; i++)
                            for (int j = 0; j < (p2->getNumVertices())/3; j++)
                                Physics::checkOverlap(*p1, *p2, (p1->getPoints()) + i * 3, (p2->getPoints())+ j * 3, e);
        }

        sBob.draw(window);
        sPat.draw(window);
        sBox.draw(window);

        groundS.draw(window, sf::Color(196, 164, 132));
        skyS.draw(window, sf::Color(135, 206, 235));
        left.draw(window);
        right.draw(window);

        // -------------------

        window.display();
    }
}