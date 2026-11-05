CC           ?= gcc
CFLAGS       ?= -Wall -Wextra -Werror -pedantic -std=c11 -O2 -Isrc
CFLAGS_DEBUG ?= -Wall -Wextra -Werror -pedantic -std=c11 -O0 -Isrc -g -DDEBUG

TEST_DIR  = tests
BUILD_DIR = build

TEST_SRCS = $(wildcard $(TEST_DIR)/*.c)
TEST_BINS = $(patsubst $(TEST_DIR)/%.c, $(BUILD_DIR)/%, $(TEST_SRCS))
SRC_SRCS  = $(wildcard src/*.c)
SRC_HDRS  = $(wildcard src/*.h)

.PHONY: all test clean

compdb: clean
	bear -- $(MAKE) test

all: test

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
