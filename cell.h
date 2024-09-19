#pragma once

#include "common.h"
#include "formula.h"

class Cell : public CellInterface {
public:
	Cell(SheetInterface& sheet);
	~Cell();

	void Set(std::string text);
	void Clear();

	Value GetValue() const override;
	std::string GetText() const override;

	std::vector<Position> GetReferencedCells() const override;

	void ClearCache();
	bool HasCache() const;

private:
	class Impl;
	class EmptyImpl;
	class TextImpl;
	class FormulaImpl;

	void InvalidateCache();
	void IsCyclicDependencies();

	std::unique_ptr<Impl> impl_;
	SheetInterface& sheet_;
	mutable std::unique_ptr<Value> cache_;
	std::vector<Position> dependencies_cells_
};