#include <3ds.h>
#include <stdio.h>

#include <box2d/box2d.h>
#include <stdlib.h>

#include <citro2d.h>

#include <memory>
#include <math.h>

#define SCREEN_WIDTH  320
#define SCREEN_HEIGHT 240

int main(int argc, char **argv)
{

    //Matrix containing the name of each key. Useful for printing when a key is pressed
	// char keysNames[32][32] = {
	// 	"KEY_A", "KEY_B", "KEY_SELECT", "KEY_START",
	// 	"KEY_DRIGHT", "KEY_DLEFT", "KEY_DUP", "KEY_DDOWN",
	// 	"KEY_R", "KEY_L", "KEY_X", "KEY_Y",
	// 	"", "", "KEY_ZL", "KEY_ZR",
	// 	"", "", "", "",
	// 	"KEY_TOUCH", "", "", "",
	// 	"KEY_CSTICK_RIGHT", "KEY_CSTICK_LEFT", "KEY_CSTICK_UP", "KEY_CSTICK_DOWN",
	// 	"KEY_CPAD_RIGHT", "KEY_CPAD_LEFT", "KEY_CPAD_UP", "KEY_CPAD_DOWN"
	// };
    const int MAX_POWER = 1000000;
    const int MIN_POWER = 100000;
    const int EXPLODE_POWER_DEFAULT = 200000;

    gfxInitDefault();
    C3D_Init(C3D_DEFAULT_CMDBUF_SIZE);
    C2D_Init(C2D_DEFAULT_MAX_OBJECTS);
    C2D_Prepare();

    cfguInit();

    C3D_RenderTarget* bottom = C2D_CreateScreenTarget(GFX_BOTTOM, GFX_LEFT);

    // Define the gravity vector.
    b2Vec2 gravity(0.0f, 150.0f);

    // Construct a world object, which will hold and simulate the rigid bodies.
    // std::unique_ptr allows for this to destroy automatically when the program exits
    std::unique_ptr<b2World> world = std::make_unique<b2World>(gravity);

    // Define the ground body.
    b2BodyDef leftWallDef,
    rightWallDef,
    ceilingDef,
    groundBodyDef;

    groundBodyDef.position.Set(0.0f, SCREEN_HEIGHT);
    leftWallDef.position.Set(0.0f, SCREEN_HEIGHT);
    rightWallDef.position.Set(SCREEN_WIDTH, 0.0f);
    ceilingDef.position.Set(0.0f, 0.0f);

    // Call the body factory which allocates memory for the ground body
    // from a pool and creates the ground box shape (also from a pool).
    // The body is also added to the world.
    b2Body *groundBody = world->CreateBody(&groundBodyDef);
    b2Body *leftWall = world->CreateBody(&leftWallDef);
    b2Body *rightWall = world->CreateBody(&rightWallDef);
    b2Body *ceiling = world->CreateBody(&ceilingDef);

    // Define the ground box shape.
    b2PolygonShape groundBox,
    wallBox,
    ceilingBox;

    // The extents are the half-widths of the box.
    groundBox.SetAsBox(SCREEN_WIDTH, 16);
    wallBox.SetAsBox(1, SCREEN_HEIGHT);
    ceilingBox.SetAsBox(SCREEN_WIDTH, 1);

    // Add the ground fixture to the ground body.
    groundBody->CreateFixture(&groundBox, 0.0f);
    leftWall->CreateFixture(&wallBox, 0.0f);
    rightWall->CreateFixture(&wallBox, 0.0f);
    ceiling->CreateFixture(&ceilingBox, 0.0f);

        C2D_SceneBegin(bottom);
    // Define the dynamic body. We set its position and call the body factory.
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f);
    b2Body* body = world->CreateBody(&bodyDef);

    // Define another box shape for our dynamic body.
    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(16.0f, 16.0f);

    // Define the dynamic body fixture.
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;

    // Set the box density to be non-zero, so it will be dynamic.
    fixtureDef.density = 2.0f;
    fixtureDef.friction = 1.0f;

    // Add the shape to the body.
    body->CreateFixture(&fixtureDef);



    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float timeStep = 1.0f / 60.0f;

    int32_t velocityIterations = 6;
    int32_t positionIterations = 2;

    b2Vec2 position = body->GetPosition();
    float angle = body->GetAngle();

    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned pointers, so be careful about your world management.

    touchPosition touch;

    C2D_TextBuf dynamicBuffer;
    dynamicBuffer = C2D_TextBufNew(4096);

    int explodePower = EXPLODE_POWER_DEFAULT;

    // Main loop
    while (aptMainLoop())
    {
        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world->Step(timeStep, velocityIterations, positionIterations);

        hidScanInput();

        u32 kDown = hidKeysDown();
        u32 kHeld = hidKeysHeld();

        if (kDown & KEY_START)
            break; // break in order to return to hbmenu

        if (kDown & KEY_DUP)
        {
            if (abs(body->GetLinearVelocity().y) <= 0.1)
            {
                body->ApplyLinearImpulse(b2Vec2(0, -MAX_POWER), body->GetWorldCenter(), true);
            }
        }
        if (kHeld & KEY_DDOWN)
        {
            explodePower = EXPLODE_POWER_DEFAULT;

            world->DestroyBody(body);

            bodyDef.position.Set(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2);
            bodyDef.linearVelocity.Set(0,0);
            touch.px = 0, touch.py = 0;

            body = world->CreateBody(&bodyDef);
            body->CreateFixture(&fixtureDef);
        }
        if (kHeld & KEY_DLEFT)
        {
            explodePower -= 2500;

            if (explodePower <= MIN_POWER)
            {
                explodePower = MIN_POWER;
            }
        }
        if (kHeld & KEY_DRIGHT)
        {
            explodePower += 2500;

            if (explodePower >= MAX_POWER)
            {
                explodePower = MAX_POWER;
            }
        }
        /*
        **
        ** Once that's done, position the bodyDefinition struct to
        ** the new touch coords, offset by half width and height of the box.
        **
        ** Since bodies need to update to this world coordinate, we destroy
        ** the body from the world and create a new one based on the new
        ** bodyDefinition and then attach the old fixture.
        */
        if (kHeld & KEY_TOUCH)
        {
            hidTouchRead(&touch);

            b2Vec2 touchLocation = {touch.px - 8, touch.py - 8};
            b2Vec2 boxPosition = body->GetPosition();
            b2Vec2 positionDif = boxPosition - touchLocation;
            float distance = positionDif.Normalize();

            b2Vec2 boxVelocity = body->GetLinearVelocity();
            
            b2Vec2 resultantVelocity = b2Vec2((boxPosition.x - touchLocation.x) * explodePower / pow(distance, 2), (boxPosition.y - touchLocation.y) * explodePower / pow(distance, 2));

            // b2Vec2 testvelocity = b2Vec2((explodePower * resultantVelocity.x) / (impulseDistance + 1),  (explodePower * resultantVelocity.y) / (impulseDistance + 1));

            body->ApplyLinearImpulse(resultantVelocity, body->GetWorldCenter(), true);

            // body->ApplyLinearImpulse(testvelocity, body->GetWorldCenter(), true);
        }

        // Now print the position and angle of the body.
        position = body->GetPosition();
        angle = body->GetAngle();

        C2D_TextBufClear(dynamicBuffer);
        C2D_Text dynamicText;

        char buffer[160];
        snprintf(buffer, sizeof(buffer), "Body Pos: (%4.2f, %4.2f) / Angle: %4.2f\nTouch at (%u, %u)\nExplosion Power: %d\nUp: Jump\nLeft/Right : Power -/+ \nDown: Reset", position.x, position.y, angle,touch.px, touch.py, explodePower);

        C2D_TextParse(&dynamicText, dynamicBuffer, buffer);
	    C2D_TextOptimize(&dynamicText);

        /* Render the scene, clear to black */
        C3D_FrameBegin(C3D_FRAME_SYNCDRAW);
        C2D_TargetClear(bottom, C2D_Color32f(0, 0, 0, 1));
        C2D_SceneBegin(bottom);

        C2D_DrawText(&dynamicText, C2D_WithColor, 10.0f, 10.0f, 0.5f, 0.5f, 0.5f, C2D_Color32f(1, 1, 1, 1));

        b2Vec2 groundPos = groundBody->GetPosition();
        C2D_DrawRectSolid(groundPos.x, groundPos.y - 16, 0.5, SCREEN_WIDTH, 32, C2D_Color32f(0, 1, 0, 1));


        b2Vec2 ceilingPos = ceiling->GetPosition();
        C2D_DrawRectSolid(ceilingPos.x, ceilingPos.y, 0.5, SCREEN_WIDTH, 8, C2D_Color32f(0, 1, 0, 1));

        /* draw the dynamic box */
        C2D_DrawRectSolid(position.x, position.y, 0.5, 16, 16, C2D_Color32f(1, 0, 0, 1));

        C3D_FrameEnd(0);
    }

    cfguExit();

    C2D_TextBufDelete(dynamicBuffer);

    // Deinit libs
    C2D_Fini();

    C3D_Fini();

    gfxExit();

    return 0;
}
