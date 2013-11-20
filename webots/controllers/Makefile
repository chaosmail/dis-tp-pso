MODS := follower  pso_chain_sup  leader

.PHONY: all
all:
	for mod in $(MODS); do \
		$(MAKE) -C $$mod; \
	done

.PHONY: clean
clean:
	for mod in $(MODS); do \
		$(MAKE) -C $$mod clean; \
	done

