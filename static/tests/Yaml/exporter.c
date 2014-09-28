/*
 * exporter.c
 *
 *  Created on: 28 sept. 2014
 *      Author: ludo6431
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <yaml.h>
#include <malloc.h>

int main() {
    FILE *fd_traj = fopen("traj.yml", "wb+");
    char tab[64];
    yaml_emitter_t traj_emitter;
    yaml_event_t traj_event;
    int i;

    assert(yaml_emitter_initialize(&traj_emitter));
    yaml_emitter_set_output_file(&traj_emitter, fd_traj);

    assert(yaml_stream_start_event_initialize(&traj_event, YAML_UTF8_ENCODING));
    assert(yaml_emitter_emit(&traj_emitter, &traj_event));

    // only one node inside a document
    assert(yaml_document_start_event_initialize(&traj_event, NULL, NULL, NULL, 1));
    assert(yaml_emitter_emit(&traj_emitter, &traj_event));
    {
        assert(yaml_sequence_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_SEQUENCE_STYLE));
        assert(yaml_emitter_emit(&traj_emitter, &traj_event));
        {
            // dictionnary / block mapping node
            assert(yaml_mapping_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_MAPPING_STYLE));
            assert(yaml_emitter_emit(&traj_emitter, &traj_event));
            for (i = 0; i < 10; i++) {
                sprintf(tab, "sid:%i", i);

                yaml_scalar_event_initialize(&traj_event, NULL, NULL, "clef", 4, 1, 0, YAML_PLAIN_SCALAR_STYLE);
                if (!yaml_emitter_emit(&traj_emitter, &traj_event)) {
                    printf("error %s (%i)\n", traj_emitter.problem, traj_emitter.error);
                }

                yaml_scalar_event_initialize(&traj_event, NULL, NULL, tab, strlen(tab), 1, 0, YAML_PLAIN_SCALAR_STYLE);
                if (!yaml_emitter_emit(&traj_emitter, &traj_event)) {
                    printf("error %s (%i)\n", traj_emitter.problem, traj_emitter.error);
                }
            }
            assert(yaml_mapping_end_event_initialize(&traj_event));
            assert(yaml_emitter_emit(&traj_emitter, &traj_event));

            assert(yaml_mapping_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_MAPPING_STYLE));
            assert(yaml_emitter_emit(&traj_emitter, &traj_event));
            {
                yaml_scalar_event_initialize(&traj_event, NULL, NULL, "clef", 4, 1, 0, YAML_PLAIN_SCALAR_STYLE);
                if (!yaml_emitter_emit(&traj_emitter, &traj_event)) {
                    printf("error %s (%i)\n", traj_emitter.problem, traj_emitter.error);
                }

                // list / sequence
                assert(yaml_sequence_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_SEQUENCE_STYLE));
                assert(yaml_emitter_emit(&traj_emitter, &traj_event));
                for (i = 0; i < 10; i++) {
                    sprintf(tab, "sid:%i", i);

                    yaml_scalar_event_initialize(&traj_event, NULL, NULL, tab, strlen(tab), 1, 0, YAML_PLAIN_SCALAR_STYLE);
                    if (!yaml_emitter_emit(&traj_emitter, &traj_event)) {
                        printf("error %s (%i)\n", traj_emitter.problem, traj_emitter.error);
                    }
                }
                assert(yaml_sequence_end_event_initialize(&traj_event));
                assert(yaml_emitter_emit(&traj_emitter, &traj_event));
            }
            assert(yaml_mapping_end_event_initialize(&traj_event));
            assert(yaml_emitter_emit(&traj_emitter, &traj_event));
        }
        assert(yaml_sequence_end_event_initialize(&traj_event));
        assert(yaml_emitter_emit(&traj_emitter, &traj_event));
    }
    assert(yaml_document_end_event_initialize(&traj_event, 1));
    assert(yaml_emitter_emit(&traj_emitter, &traj_event));

    assert(yaml_stream_end_event_initialize(&traj_event));
    if (!yaml_emitter_emit(&traj_emitter, &traj_event)) {
        printf("error %s (%i)\n", traj_emitter.problem, traj_emitter.error);
    }

    assert(yaml_emitter_flush(&traj_emitter));

    yaml_emitter_delete(&traj_emitter);

    if (fd_traj)
        fclose(fd_traj);

    malloc_stats();

    return 0;
}
