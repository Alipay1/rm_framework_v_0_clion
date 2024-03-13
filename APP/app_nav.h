//
// Created by zhpwa on 2024/1/12.
//

#ifndef APP_NAV_H_
#define APP_NAV_H_

#define DEFALT_DGR_0 3514
#define DEFALT_DGR_1 1714
#define DEFALT_DGR_2 3751
#define DEFALT_DGR_3 4239

typedef struct {
    struct {
        float velocity;
        float direction;
        float direction_dgr;
    } vector;

    float angular_velocity;

    double Vx;
    double Vy;
    double Vw;

    float V[4];
    float theta[4];
} app_nav_t;

float map_degree_to_8191(float input);

void nav_main(void);

app_nav_t *get_navigation_p(void);

#endif //APP_NAV_H_
