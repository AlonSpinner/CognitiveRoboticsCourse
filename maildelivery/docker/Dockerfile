FROM karpase/planning

WORKDIR /usr/src/

COPY problem.pddl ./

COPY domain.pddl ./

RUN ulimit -t 10

RUN /planners/temporal/optic/release/optic/optic-clp -N ./domain.pddl ./problem.pddl > plan.txt

CMD ["sleep", "infinity"]
