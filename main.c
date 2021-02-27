#include "linmath.h"

#define FAST_OBJ_IMPLEMENTATION
#include "fast_obj.h"

#include <stdio.h>

void accumulate_volume_and_weighted_centroid(float *volume,
                                             vec3 weighted_centroid_sum,
                                             vec3 v1, vec3 v2, vec3 v3) {

    // printf("triangle {%.3f %.3f %.3f} {%.3f %.3f %.3f} {%.3f %.3f %.3f}\n",
    //        v1[0], v1[1], v1[2], v2[0], v2[1], v2[2], v3[0], v3[1], v3[2]);

    // implicit 4th vertex is origin
    const vec3 tet_centroid = { 0.25f * (v1[0] + v2[0] + v3[0]),
                                0.25f * (v1[1] + v2[1] + v3[1]),
                                0.25f * (v1[2] + v2[2] + v3[2]) };

    vec3 v2xv3;
    vec3_mul_cross(v2xv3, v2, v3);
    const float tet_signed_volume = vec3_mul_inner(v1, v2xv3) * 0.16666667f;
        // 1/6 scalar triple product of vertices

    *volume += tet_signed_volume;

    vec3 tet_weighted_centroid;
    vec3_scale(tet_weighted_centroid, tet_centroid, tet_signed_volume);

    vec3_add(weighted_centroid_sum,
             weighted_centroid_sum, tet_weighted_centroid);
}

void volume_and_weighted_centroid_sum(float *volume,
                                      vec3 weighted_centroid_sum,
                                      fastObjMesh *mesh) {

    unsigned int index_offset = 0;

    for (unsigned int f = 0; f != mesh->face_count; ++f) {
        unsigned int num_vertices = mesh->face_vertices[f];

        if (num_vertices != 3) {
            printf("unsupported mesh: triangle meshes only\n");
            exit(EXIT_FAILURE);
        }

        unsigned int p1 = mesh->indices[index_offset + 0].p;
        unsigned int p2 = mesh->indices[index_offset + 1].p;
        unsigned int p3 = mesh->indices[index_offset + 2].p;

        accumulate_volume_and_weighted_centroid(volume, weighted_centroid_sum,
                                                mesh->positions + (3 * p1),
                                                mesh->positions + (3 * p2),
                                                mesh->positions + (3 * p3));

        index_offset += num_vertices;
    }
}

int main(int argc, char **argv) {

    if (argc < 2) {
        printf("usage: %s thing.obj\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    fastObjMesh *mesh = fast_obj_read(argv[1]);

    if (mesh == NULL) {
        printf("mesh read failure\n");
        exit(EXIT_FAILURE);
    }

    float volume = 0.f;
    vec3 weighted_centroid_sum = { 0.f, 0.f, 0.f };

    volume_and_weighted_centroid_sum(&volume, weighted_centroid_sum, mesh);
    vec3 centre_of_mass;
    vec3_scale(centre_of_mass, weighted_centroid_sum, 1.f / volume);

    printf("volume: %.3f\n"
           "centre of mass: {%.3f, %.3f, %.3f}\n",
           volume,
           centre_of_mass[0], centre_of_mass[1], centre_of_mass[2]);

    fast_obj_destroy(mesh);
}
