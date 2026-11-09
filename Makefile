CC           ?= gcc
CFLAGS       ?= -Wall -Wextra -Werror -pedantic -std=c11 -O2 -Isrc -isystem vendor
CFLAGS_DEBUG ?= -Wall -Wextra -Werror -pedantic -std=c11 -O0 -Isrc -isystem vendor -g -DDEBUG -fsanitize=address,undefined

TEST_DIR  = tests
BUILD_DIR = build

TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_BINS = $(patsubst $(TEST_DIR)/%.c, $(BUILD_DIR)/%, $(TEST_SRCS))
SRC_SRCS  = $(wildcard src/*.c)
SRC_HDRS  = $(wildcard src/*.h)
ALL_C     = $(wildcard src/*.c src/*.h $(TEST_DIR)/*.c $(TEST_DIR)/*.h)

.PHONY: all test clean compdb lint

all: lint test

lint:
	@tabs=$$(grep -Pn '\t' $(ALL_C) 2>/dev/null); if [ -n "$$tabs" ]; then                     \
		echo "\033[1;31mfail\033[0m: hard tabs found:";                                    \
		echo "$$tabs";									   \
		exit 1;									     	   \
	fi
	@spaces=$$(grep -Pn '[ ]+$$' $(ALL_C) 2>/dev/null); if [ -n "$$spaces" ]; then             \
		echo "\033[1;31mfail\033[0m: trailing whitespaces found:";                         \
		echo "$$spaces";								   \
		exit 1;										   \
	fi
	@for f in $(wildcard src/*.h $(TEST_DIR)/*.h); do                                          \
		head -n 1 "$$f" | grep -q "#pragma once" || 					   \
		(echo: "\033[1;31mfail\033[0m: missing #pragma once in $$f" && exit 1); 				   \
	done

compdb: clean
	bear -- $(MAKE) $(TEST_BINS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%: $(TEST_DIR)/%.c $(SRC_SRCS) $(SRC_HDRS) | $(BUILD_DIR)
	$(CC) $(CFLAGS_DEBUG) $(filter %.c, $^) -o $@ -lm

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do 								   \
		echo "$$t";									   \
		$$t || exit 1;									   \
	done

clean:
	rm -rf $(BUILD_DIR)
