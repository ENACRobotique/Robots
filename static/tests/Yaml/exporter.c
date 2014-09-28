/*
 * exporter.c
 *
 *  Created on: 28 sept. 2014
 *      Author: ludo6431
 */

#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <yaml.h>

int main() {
    FILE *fd_traj;
    char tab[64];
    yaml_emitter_t traj_emitter;
    yaml_event_t traj_event;
    int i;

#define ERR_CHECK(ret) do { if(!(ret)){ printf("error %s (%i)\n", traj_emitter.problem, traj_emitter.error); } } while(0)

    fd_traj = fopen("traj.yml", "wb+");
    if(!fd_traj){
        perror("fopen");
        exit(1);
    }

    // initialize the writer, object used in each call
    ERR_CHECK(yaml_emitter_initialize(&traj_emitter));
    yaml_emitter_set_output_file(&traj_emitter, fd_traj);

    // start document
    ERR_CHECK(yaml_stream_start_event_initialize(&traj_event, YAML_UTF8_ENCODING));
    ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));

    // only one node inside a document
    ERR_CHECK(yaml_document_start_event_initialize(&traj_event, NULL, NULL, NULL, 1));
    ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
    {
        // starting a list
        ERR_CHECK(yaml_sequence_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_SEQUENCE_STYLE));
        ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
        {
            // first element of the list
            // mapping node
            ERR_CHECK(yaml_mapping_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_MAPPING_STYLE));
            ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
            for (i = 0; i < 10; i++) {
                sprintf(tab, "sid:%i", i);

                // key
                ERR_CHECK(yaml_scalar_event_initialize(&traj_event, NULL, NULL, (yaml_char_t*)"clef", 4, 1, 0, YAML_PLAIN_SCALAR_STYLE));
                ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));

                // value
                ERR_CHECK(yaml_scalar_event_initialize(&traj_event, NULL, NULL, (yaml_char_t*)tab, strlen(tab), 1, 0, YAML_PLAIN_SCALAR_STYLE));
                ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
            }
            ERR_CHECK(yaml_mapping_end_event_initialize(&traj_event));
            ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));

            // second element of the list
            // mapping node
            ERR_CHECK(yaml_mapping_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_MAPPING_STYLE));
            ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
            {
                // key
                ERR_CHECK(yaml_scalar_event_initialize(&traj_event, NULL, NULL, (yaml_char_t*)"clef", 4, 1, 0, YAML_PLAIN_SCALAR_STYLE));
                ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));

                // value
                // list / sequence
                ERR_CHECK(yaml_sequence_start_event_initialize(&traj_event, NULL, NULL, 1, YAML_BLOCK_SEQUENCE_STYLE));
                ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
                for (i = 0; i < 10; i++) {
                    sprintf(tab, "sid:%i", i);

                    ERR_CHECK(yaml_scalar_event_initialize(&traj_event, NULL, NULL, (yaml_char_t*)tab, strlen(tab), 1, 0, YAML_PLAIN_SCALAR_STYLE));
                    ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
                }
                ERR_CHECK(yaml_sequence_end_event_initialize(&traj_event));
                ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
            }
            ERR_CHECK(yaml_mapping_end_event_initialize(&traj_event));
            ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
        }
        ERR_CHECK(yaml_sequence_end_event_initialize(&traj_event));
        ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));
    }
    // end of document
    ERR_CHECK(yaml_document_end_event_initialize(&traj_event, 1));
    ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));

    ERR_CHECK(yaml_stream_end_event_initialize(&traj_event));
    ERR_CHECK(yaml_emitter_emit(&traj_emitter, &traj_event));

    ERR_CHECK(yaml_emitter_flush(&traj_emitter));

    yaml_emitter_delete(&traj_emitter);
    fclose(fd_traj);

    malloc_stats();

    return 0;

#undef ERR_CHECK
}
